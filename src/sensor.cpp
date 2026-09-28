// SPDX-License-Identifier: MIT
#include "leptrino_force_torque_ros_driver/sensor.hpp"

#include <poll.h>

#include <cerrno>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace leptrino_force_torque_ros_driver
{
namespace
{
constexpr std::size_t receive_chunk_size = 4096;
}  // namespace

bool validTimeout(double seconds)
{
  const long double ticks =
    static_cast<long double>(seconds) * Clock::period::den / Clock::period::num;
  return std::isfinite(seconds) && seconds > 0 && ticks >= 1 &&
    ticks < static_cast<long double>(Clock::duration::max().count()) / 4;
}

Sensor::Sensor(const std::string & port, double timeout, int retries, Runtime runtime)
: port_(port), retries_(retries), runtime_(std::move(runtime))
{
  if (!validTimeout(timeout) || retries < 0 || retries == std::numeric_limits<int>::max()) {
    throw std::invalid_argument("invalid command timeout or retries");
  }

  timeout_ = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(timeout));
}

Sensor::~Sensor()
{
  try {
    stop();
  } catch (...) {
    // Explicit stop() reports errors; unwinding still releases the port.
  }
}

bool Sensor::receive(const Frame & frame, Deadline deadline)
{
  const short events = port_.wait(POLLIN, deadline);
  bool consumed = false;

  if (events & POLLIN) {
    uint8_t bytes[receive_chunk_size];
    const ssize_t count = port_.read(bytes, sizeof(bytes));
    if (count > 0) {
      ReceiveStamp stamp;
      stamp.steady = Clock::now();
      stamp.ros_nanoseconds = runtime_.timestamp();
      parser_.feed(bytes, static_cast<std::size_t>(count), frame, stamp);
      if (chunk_done_) {
        chunk_done_(stamp);
      }
      consumed = true;
    } else if (count < 0 && errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) {
      throw std::runtime_error(std::string("serial read: ") + std::strerror(errno));
    }
  }

  if (events & (POLLERR | POLLHUP | POLLNVAL)) {
    throw std::runtime_error("serial port disconnected");
  }

  return consumed;
}

bool Sensor::accept(uint8_t code, const uint8_t * data, std::size_t size)
{
  switch (code) {
    case command_code::get_info:
      return decodeProduct(data, size, product_);
    case command_code::get_limits:
      return conversionFactors(data, size, factors_);
    case command_code::get_filter:
      if (!validFilter(data, size)) {
        return false;
      }
      filter_ = data[field::payload];
      return true;
    default:
      return true;
  }
}

void Sensor::command(uint8_t code, int retries)
{
  /// State of the current command attempt.
  enum class State
  {
    waiting,
    success,
    retry,
    fatal
  };

  const uint8_t request[] = {header_size, sensor_id, code, 0};
  std::array<uint8_t, max_wire> wire;
  const auto length = encode(request, sizeof(request), wire);

  for (int attempt = 0; attempt <= retries; ++attempt) {
    parser_.reset();
    const auto now = Clock::now();
    if (now > Deadline::max() - timeout_) {
      throw std::runtime_error("command deadline overflow");
    }
    const auto deadline = now + timeout_;

    std::size_t written = 0;
    while (written < length && running() && Clock::now() < deadline) {
      if (code == command_code::start) {
        start_sent_ = true;
      }
      const auto count = port_.write(wire.data() + written, length - written);
      if (count > 0) {
        written += static_cast<std::size_t>(count);
      } else if (count < 0 && errno == EINTR) {
        continue;
      } else if (count == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
        if (port_.wait(POLLOUT, deadline) & (POLLERR | POLLHUP | POLLNVAL)) {
          throw std::runtime_error("serial write poll failed");
        }
      } else {
        throw std::runtime_error(std::string("serial write: ") + std::strerror(errno));
      }
    }

    State state = State::waiting;
    std::string reason = "timeout";

    const Frame frame =
      [&](Event event, const uint8_t * data, std::size_t size, const ReceiveStamp & stamp) {
        const bool sample = event == Event::frame &&
          validate(data, size, command_code::start) == Response::measurement;

        if (state == State::success && code == command_code::stop) {
          if (sample) {
            state = State::retry;
            reason = "stream continued after STOP";
          }
          return;  // Quarantine all remaining responses after STOP.
        }

        if (state == State::waiting) {
          if (event == Event::nak) {
            state = State::retry;
            reason = "NAK";
            return;
          }

          const auto response = validate(data, size, code);
          if (response == Response::fatal || response == Response::retry) {
            state = response == Response::fatal ? State::fatal : State::retry;
            reason = "sensor result " + std::to_string(data[field::result]);
          } else if (response == Response::success) {
            if (accept(code, data, size)) {
              state = Clock::now() < deadline ? State::success : State::waiting;
              if (state == State::success && code == command_code::start) {
                streaming_ = true;
                if (started_) {
                  started_();
                }
              }
            } else {
              state = State::retry;
              reason = "invalid response content";
            }
          } else if (
            response == Response::invalid && size >= header_size && data[field::command] == code)
          {
            reason = "invalid response length or header";
          }
        }

        if (sample) {
          if (streaming_ && measurement_) {
            measurement_(data, stamp);
          } else if (code != command_code::stop && code != command_code::start) {
            throw std::runtime_error("unexpected measurement after STOP");
          }
        }
      };

    while (written == length && state == State::waiting && running() && Clock::now() < deadline) {
      receive(frame, deadline);
    }

    if (state == State::success && code == command_code::stop) {
      bool idle = false;
      while (state == State::success && running() && Clock::now() < deadline) {
        if (!receive(frame, deadline)) {
          idle = true;
          break;
        }
      }
      if (state == State::success && (!idle || Clock::now() >= deadline)) {
        state = State::waiting;
      }
      parser_.reset();
    }

    if (state == State::success) {
      return;
    }
    if (!running()) {
      throw std::runtime_error("sensor operation cancelled");
    }

    std::ostringstream message;
    message << "Command 0x" << std::hex << unsigned(code) << std::dec << ", attempt " << attempt + 1
            << ": " << reason;
    if (state == State::fatal || attempt == retries) {
      throw std::runtime_error(message.str());
    }
    runtime_.warn(message.str());
  }
}

void Sensor::initialize()
{
  command(command_code::stop, retries_);
  command(command_code::get_info, retries_);
  command(command_code::get_limits, retries_);
  if (product_.protocol == Protocol::v113) {
    command(command_code::get_filter, retries_);
  }

  initialized_ = true;
}

void Sensor::run(
  const Measurement & measurement, const ChunkDone & chunk_done,
  const std::function<void()> & started)
{
  if (!initialized_) {
    throw std::logic_error("sensor is not initialized");
  }
  if (start_sent_) {
    throw std::logic_error("stop() must succeed before run() can be called again");
  }

  measurement_ = measurement;
  chunk_done_ = chunk_done;
  started_ = started;
  command(command_code::start, retries_);

  const Frame frame = [&](
                        Event event, const uint8_t * data, std::size_t size,
                        const ReceiveStamp & stamp) {
    if (event == Event::frame && validate(data, size, command_code::start) == Response::measurement)
    {
      measurement_(data, stamp);
    }
  };

  while (running()) {
    receive(frame, Deadline::max());
  }
}

void Sensor::stop()
{
  streaming_ = false;
  measurement_ = {};
  chunk_done_ = {};
  started_ = {};

  if (!start_sent_) {
    return;
  }

  ignore_runtime_stop_ = true;  // Temporarily ignore runtime_.running() to complete the STOP command.
  try {
    command(command_code::stop, retries_);
    start_sent_ = false;
  } catch (...) {
    ignore_runtime_stop_ = false;
    throw;
  }
  ignore_runtime_stop_ = false;
}
}  // namespace leptrino_force_torque_ros_driver
