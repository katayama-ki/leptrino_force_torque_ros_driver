// SPDX-License-Identifier: MIT
#ifndef LEPTRINO_WRENCH_PROTOCOL_HPP_
#define LEPTRINO_WRENCH_PROTOCOL_HPP_

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>

namespace leptrino_force_torque_ros_driver
{
using Clock = std::chrono::steady_clock;
using Deadline = Clock::time_point;

/// Host timestamps captured after a read; all frames completed in that read share them.
struct ReceiveStamp
{
  uint64_t ros_nanoseconds = 0;
  Deadline steady{};
};

// Protocol values shared by v1.13 and v1.31. Offsets refer to the unescaped body.
namespace control
{
constexpr uint8_t dle = 0x10;
constexpr uint8_t stx = 0x02;
constexpr uint8_t etx = 0x03;
constexpr uint8_t nak = 0x15;
}  // namespace control

namespace command_code
{
constexpr uint8_t get_info = 0x2a;
constexpr uint8_t get_limits = 0x2b;
constexpr uint8_t start = 0x32;
constexpr uint8_t stop = 0x33;
constexpr uint8_t get_filter = 0xb6;
}  // namespace command_code

namespace result_code
{
constexpr uint8_t success = 0x00;
constexpr uint8_t state_error = 0x04;  // Retryable; other sensor error results are fatal.
}  // namespace result_code

namespace field
{
constexpr std::size_t length = 0;
constexpr std::size_t sensor = 1;
constexpr std::size_t command = 2;
constexpr std::size_t result = 3;
constexpr std::size_t payload = 4;

constexpr std::size_t model = payload;
constexpr std::size_t model_size = 16;
constexpr std::size_t serial = model + model_size;
constexpr std::size_t serial_size = 8;
constexpr std::size_t firmware = serial + serial_size;
constexpr std::size_t firmware_size = 4;
constexpr std::size_t output_rate = firmware + firmware_size;
constexpr std::size_t output_rate_size = 6;

constexpr std::size_t measurement_status = 18;
}  // namespace field

namespace axis
{
enum : std::size_t
{
  fx,
  fy,
  fz,
  mx,
  my,
  mz,
  count
};
}  // namespace axis

namespace status_bit
{
constexpr uint8_t correction_error = 0x01;
constexpr uint8_t sensor_error = 0x02;
constexpr uint8_t overload = 0x04;
constexpr uint8_t errors = correction_error | sensor_error | overload;
constexpr std::size_t error_count = 3;
}  // namespace status_bit

namespace filter_code
{
constexpr uint8_t off = 0;
constexpr uint8_t hz10 = 1;
constexpr uint8_t hz100 = 2;
constexpr uint8_t hz200 = 3;
}  // namespace filter_code

constexpr uint8_t sensor_id = 0xff;
constexpr std::size_t header_size = field::payload;
constexpr std::size_t force_value_size = 2;
constexpr std::size_t limit_value_size = 4;

constexpr std::size_t product_v113_size = field::output_rate;
constexpr std::size_t product_v131_size = field::output_rate + field::output_rate_size;
constexpr std::size_t limits_size = header_size + axis::count * limit_value_size;
constexpr std::size_t measurement_size = 20;
constexpr std::size_t filter_size = 8;

constexpr double counts_at_rated_load = 10000.0;
constexpr double raw_to_fraction = 1.0 / counts_at_rated_load;

constexpr std::size_t max_body = 128;
// DLE/STX + escaped body (at most two bytes per byte) + DLE/ETX/BCC.
constexpr std::size_t max_wire = 2 + 2 * max_body + 3;

/// Parser output: a length/BCC-checked body or a standalone negative acknowledgement.
enum class Event
{
  frame,
  nak
};

/// Incrementally parse bounded frames, preserving partial frames across reads.
class Parser
{
public:
  /// Discard the current partial frame and resume searching for a frame boundary.
  void reset()
  {
    state_ = State::idle;
    size_ = 0;
    bcc_ = 0;
  }

  /// Dispatch callback(event, body, size, stamp) synchronously for each frame or NAK.
  /// Body storage is borrowed only for the callback; NAK uses nullptr and size zero.
  template <class Callback>
  void feed(
    const uint8_t * bytes, std::size_t count, Callback && callback,
    const ReceiveStamp & stamp = ReceiveStamp())
  {
    for (std::size_t i = 0; i < count; ++i) {
      const uint8_t c = bytes[i];
      switch (state_) {
        case State::idle:
          if (c == control::dle) {
            state_ = State::marker;
          }
          break;

        case State::marker:
          if (c == control::stx) {
            begin();
          } else if (c == control::nak) {
            reset();
            callback(Event::nak, nullptr, 0, stamp);
          } else if (c != control::dle) {
            reset();
          }
          break;

        case State::body:
          if (c == control::dle) {
            state_ = State::escape;
          } else {
            append(c);
          }
          break;

        case State::escape:
          if (c == control::dle) {
            state_ = State::body;
            append(c);
          } else if (c == control::etx) {
            bcc_ ^= c;
            state_ = State::bcc;
          } else if (c == control::stx) {
            begin();
          } else {
            reset();
            if (c == control::nak) {
              callback(Event::nak, nullptr, 0, stamp);
            }
          }
          break;

        case State::bcc: {
          const bool valid = c == bcc_ && size_ >= header_size && body_[field::length] == size_;
          const std::size_t length = size_;

          reset();
          if (valid) {
            callback(Event::frame, body_.data(), length, stamp);
          } else if (c == control::dle) {
            state_ = State::marker;
          }
          break;
        }
      }
    }
  }

private:
  enum class State
  {
    idle,
    marker,
    body,
    escape,
    bcc
  };

  void begin()
  {
    size_ = 0;
    bcc_ = 0;
    state_ = State::body;
  }

  void append(uint8_t c)
  {
    if (size_ == max_body) {
      reset();
      return;
    }

    body_[size_++] = c;
    bcc_ ^= c;
  }

  State state_ = State::idle;
  std::array<uint8_t, max_body> body_{};
  std::size_t size_ = 0;
  uint8_t bcc_ = 0;
};

/// Classify a checked body against the pending command, separating ACKs from measurements.
enum class Response
{
  unrelated,
  invalid,
  success,
  measurement,
  retry,
  fatal
};

/// Supported product-information response layouts.
enum class Protocol
{
  v113,
  v131
};

/// Validated product fields with NUL and space padding removed.
struct ProductInfo
{
  Protocol protocol = Protocol::v131;
  std::string model;
  std::string serial;
  std::string firmware;
  std::string output_rate;
};

/// Encode framing, escaping and BCC; return the wire length, or zero for an invalid body length.
std::size_t encode(const uint8_t * body, std::size_t size, std::array<uint8_t, max_wire> & wire);
/// Decode two little-endian bytes as a signed measurement count.
int16_t signed16(const uint8_t * bytes);
/// Decode four little-endian IEEE-754 bytes without requiring aligned storage.
float float32(const uint8_t * bytes);

/// Check header, result and command-specific length after parser framing/BCC validation.
Response validate(const uint8_t * body, std::size_t size, uint8_t expected);
/// Decode supported product replies; leave info unchanged on validation failure.
bool decodeProduct(const uint8_t * body, std::size_t size, ProductInfo & info);
/// Commit all six rated-limit / 10000 factors only if all limits and factors are finite and
/// positive.
bool conversionFactors(
  const uint8_t * body, std::size_t size, std::array<double, axis::count> & factors);
/// Validate the filter reply, including its setting and reserved bytes.
bool validFilter(const uint8_t * body, std::size_t size);

/// Convert a validated measurement to Fx/Fy/Fz/Mx/My/Mz using validated factors.
std::array<double, axis::count> decodeWrench(
  const uint8_t * body, const std::array<double, axis::count> & factors);

/// Extract error bits from a validated measurement; external-input bit 5 is excluded.
inline uint8_t statusBits(const uint8_t * body)
{
  return body[field::measurement_status] & status_bit::errors;
}
}  // namespace leptrino_force_torque_ros_driver

#endif  // LEPTRINO_WRENCH_PROTOCOL_HPP_
