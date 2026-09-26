// SPDX-License-Identifier: MIT
#include "leptrino_force_torque_ros_driver/publication.hpp"

#include <cassert>
#include <vector>

using namespace leptrino_force_torque_ros_driver;

namespace
{
std::vector<uint8_t> encoded(const std::vector<uint8_t> & body)
{
  std::array<uint8_t, max_wire> wire;
  const auto size = encode(body.data(), body.size(), wire);
  return {wire.begin(), wire.begin() + size};
}

std::vector<uint8_t> measurement(uint16_t value, uint8_t status = 0)
{
  std::vector<uint8_t> body(measurement_size, 0);
  body[0] = measurement_size;
  body[1] = sensor_id;
  body[2] = command_code::start;
  body[field::payload] = value & 0xff;
  body[field::payload + 1] = value >> 8;
  body[field::measurement_status] = status;
  return encoded(body);
}

void append(std::vector<uint8_t> & bytes, const std::vector<uint8_t> & frame)
{
  bytes.insert(bytes.end(), frame.begin(), frame.end());
}

struct Receiver
{
  explicit Receiver(double rate) : publication(rate)
  {
  }

  void read(const std::vector<uint8_t> & bytes, int milliseconds)
  {
    ReceiveStamp stamp;
    stamp.steady = Deadline{} + std::chrono::milliseconds(milliseconds);
    parser.feed(
      bytes.data(), bytes.size(),
      [&](Event event, const uint8_t * data, std::size_t size, const ReceiveStamp & received) {
        if (
          event == Event::frame &&
          validate(data, size, command_code::start) == Response::measurement)
        {
          warnings |= status.warnings(statusBits(data), received.steady);
          publication.update(data);
        }
      },
      stamp);
    const auto * latest = publication.finishChunk(stamp.steady);
    if (latest) {
      values.push_back(signed16(latest + field::payload));
    }
    assert(publication.finishChunk(stamp.steady) == nullptr);
  }

  Parser parser;
  ChunkPublication publication;
  StatusThrottle status;
  uint8_t warnings = 0;
  std::vector<int16_t> values;
};

void checkSplitAndInvalidFrames(double rate)
{
  Receiver receiver(rate);
  auto chunk = encoded({4, sensor_id, command_code::start, 0});
  append(chunk, measurement(16, status_bit::overload));
  append(chunk, measurement(32));
  auto damaged = measurement(64);
  damaged.back() ^= 1;
  append(chunk, damaged);
  append(chunk, encoded({4, sensor_id, command_code::stop, 0}));

  // The next frame overwrites parser storage and ends halfway through a DLE escape.
  const auto next = measurement(0x1010);
  const std::size_t split = 7;
  assert(next[split - 1] == control::dle && next[split] == control::dle);
  chunk.insert(chunk.end(), next.begin(), next.begin() + split);
  receiver.read(chunk, 0);
  assert(receiver.values == std::vector<int16_t>{32});
  assert(receiver.warnings == status_bit::overload);

  receiver.read({next.begin() + split, next.end()}, 2);
  assert((receiver.values == std::vector<int16_t>{32, 0x1010}));
  receiver.read(damaged, 4);
  assert(receiver.values.size() == 2);

  // A damaged chunk must not prevent the next valid measurement from being published.
  receiver.read(measurement(48), 6);
  assert(receiver.values.back() == 48 && receiver.values.size() == 3);
}

void checkRateAndFreshness()
{
  Receiver receiver(500);
  receiver.read(measurement(1), 0);
  receiver.read(measurement(2, status_bit::sensor_error), 1);
  assert(receiver.values == std::vector<int16_t>{1});
  assert(receiver.warnings == status_bit::sensor_error);
  receiver.read(encoded({4, sensor_id, command_code::start, 0}), 2);
  assert(receiver.values.size() == 1);  // Never publish the rate-limited candidate later.

  auto chunk = measurement(3);
  append(chunk, measurement(4));
  receiver.read(chunk, 3);
  receiver.read(measurement(5), 4);  // Retain the original 2 ms phase.
  receiver.read(measurement(6), 100);
  receiver.read(measurement(7), 101);  // No catch-up after the long gap.
  receiver.read(measurement(8), 102);
  assert((receiver.values == std::vector<int16_t>{1, 4, 5, 6, 8}));
}

void checkBurst(double rate)
{
  Receiver receiver(rate);
  for (int i = 0; i < 3; ++i) {
    std::vector<uint8_t> chunk;
    for (int j = 0; j < 12; ++j) {
      append(chunk, measurement(i * 12 + j));
    }
    receiver.read(chunk, i * 10);
    assert(receiver.values.size() == static_cast<std::size_t>(i + 1));
    assert(receiver.values.back() == i * 12 + 11);
  }
}
}  // namespace

int main()
{
  for (double rate : {0.0, 500.0}) {
    checkSplitAndInvalidFrames(rate);
    checkBurst(rate);
  }
  checkRateAndFreshness();
}
