// SPDX-License-Identifier: MIT
#include "leptrino_force_torque_ros_driver/sensor.hpp"

#include <poll.h>
#include <pty.h>
#include <unistd.h>

#include <atomic>
#include <cassert>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "leptrino_force_torque_ros_driver/publication.hpp"

using namespace leptrino_force_torque_ros_driver;

/// A PTY exercises the real poll/read/write path, without a general mock-I/O framework.
struct Device
{
  int master;
  int slave;
  char path[128];
  bool legacy;
  int mode;
  std::atomic<bool> done{false};
  std::thread worker;
  std::vector<int> commands;
  int information_requests = 0;
  int limit_requests = 0;

  Device(bool old, int fault) : legacy(old), mode(fault)
  {
    assert(openpty(&master, &slave, path, nullptr, nullptr) == 0);

    worker = std::thread([this] {
      Parser parser;
      while (!done) {
        pollfd fd{master, POLLIN, 0};
        if (poll(&fd, 1, 10) <= 0) {
          continue;
        }

        uint8_t bytes[512];
        const auto n = read(master, bytes, sizeof(bytes));
        if (n <= 0) {
          continue;
        }

        parser.feed(bytes, n, [&](Event, const uint8_t * data, std::size_t, const ReceiveStamp &) {
          const int code = data[2];
          commands.push_back(code);
          if (code == 0x2a) {
            ++information_requests;
            if (mode == 2) {
              return;  // Silence is not a legacy response.
            }

            std::vector<uint8_t> reply{uint8_t(legacy ? 32 : 38), 255, 0x2a, 0};
            const std::string fields = "PFS055YA251U6S  2606008 40091200  ";
            reply.insert(reply.end(), fields.begin(), fields.begin() + (legacy ? 28 : 34));
            if (mode == 1 && information_requests == 1) {
              reply[20] = 'x';
            }

            send(reply);
          } else if (code == 0x2b) {
            ++limit_requests;
            std::vector<uint8_t> reply(28, 0);
            reply[0] = 28;
            reply[1] = 255;
            reply[2] = 0x2b;
            for (int i = 0; i < 6; ++i) {
              reply[4 + 4 * i + 2] = 0x7a;
              reply[4 + 4 * i + 3] = 0x43;
            }
            if (mode == 1 && limit_requests == 1) {
              // Inf
              reply[26] = 0x80;
              reply[27] = 0x7f;
            }

            send(reply);
          } else if (code == 0xb6) {
            if (mode == 3) {
              send({4, 255, 0xb6, 2});
            } else {
              send({8, 255, 0xb6, 0, 2, 0, 0, 0});
            }
          } else if (code == 0x32) {
            if (mode == 4) {
              return;  // Lost START response must still cause STOP.
            }

            std::vector<uint8_t> sample(20, 0);
            sample[0] = 20;
            sample[1] = 255;
            sample[2] = 0x32;
            sample[4] = 16;
            sample[19] = 0xff;

            auto packet = encoded({4, 255, 0x32, 0});
            const auto frame = encoded(sample);
            packet.insert(packet.end(), frame.begin(), frame.end());
            sample[4] = 17;
            const auto latest = encoded(sample);
            packet.insert(packet.end(), latest.begin(), latest.end());
            assert(write(master, packet.data(), packet.size()) == ssize_t(packet.size()));
          } else {
            send({4, 255, uint8_t(code), 0});
          }
        });
      }
    });
  }

  ~Device()
  {
    finish();
    close(master);
    close(slave);
  }

  void finish()
  {
    done = true;
    if (worker.joinable()) {
      worker.join();
    }
  }

  static std::vector<uint8_t> encoded(const std::vector<uint8_t> & body)
  {
    std::array<uint8_t, max_wire> wire;
    const auto n = encode(body.data(), body.size(), wire);
    return {wire.begin(), wire.begin() + n};
  }

  void send(const std::vector<uint8_t> & body)
  {
    const auto wire = encoded(body);
    assert(write(master, wire.data(), wire.size()) == ssize_t(wire.size()));
  }
};

int main()
{
  for (int mode = 0; mode <= 4; ++mode) {
    for (bool legacy : {false, true}) {
      if (mode == 3 && !legacy) {
        continue;
      }

      Device device(legacy, mode);
      bool alive = true;
      std::string failure;
      int samples = 0;
      int publications = 0;
      int latest_value = 0;
      int chunk_samples = 0;
      ChunkPublication publication(0);
      int warnings = 0;
      const auto started = Clock::now();
      Runtime runtime{
        [&] { return alive && Clock::now() - started < std::chrono::seconds(2); },
        [] { return uint64_t(123); },
        [&](const std::string &) { ++warnings; },
      };

      {
        Sensor sensor(device.path, .05, (mode == 1 || mode == 3) ? 1 : 0, runtime);
        try {
          sensor.initialize();
          assert(sensor.product().protocol == (legacy ? Protocol::v113 : Protocol::v131));
          assert(sensor.filter() == (legacy ? 2 : -1));
          sensor.run(
            [&](const uint8_t * data, const ReceiveStamp & stamp) {
              assert(stamp.ros_nanoseconds == 123 && signed16(data + 4) == 16 + samples);
              ++samples;
              ++chunk_samples;
              publication.update(data);
              alive = samples < 2;
            },
            [&](const ReceiveStamp & stamp) {
              const auto * data = publication.finishChunk(stamp.steady);
              assert((data != nullptr) == (chunk_samples > 0));
              if (data) {
                latest_value = signed16(data + 4);
                assert(latest_value == 15 + samples && stamp.ros_nanoseconds == 123);
                ++publications;
              }
              chunk_samples = 0;
            });
        } catch (const std::runtime_error & error) {
          failure = error.what();
        }

        sensor.stop();
        assert(sensor.close());
      }

      device.finish();
      const char * expected_errors[] = {
        "", "", "Command 0x2a, attempt 1: timeout", "Command 0xb6, attempt 1: sensor result 2",
        "Command 0x32, attempt 1: timeout"};
      if (failure != expected_errors[mode]) {
        std::cerr << "mode=" << mode << " legacy=" << legacy << ": expected '"
                  << expected_errors[mode] << "', got '" << failure << "'" << std::endl;
      }
      assert(failure == expected_errors[mode]);
      assert(samples == (mode < 2 ? 2 : 0));
      assert(publications == 0 ? mode >= 2 : publications <= samples && latest_value == 17);
      assert(warnings == (mode == 1 ? 2 : 0));

      int filters = 0;
      int stops = 0;
      for (int code : device.commands) {
        filters += code == 0xb6;
        stops += code == 0x33;
      }
      assert(filters == (legacy && mode != 2 ? 1 : 0));
      assert(stops == (mode < 2 || mode == 4 ? 2 : 1));
    }
  }
}
