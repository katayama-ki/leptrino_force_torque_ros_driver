// SPDX-License-Identifier: MIT
#include "leptrino_force_torque_ros_driver/protocol.hpp"

namespace leptrino_force_torque_ros_driver
{
std::size_t encode(const uint8_t * body, std::size_t size, std::array<uint8_t, max_wire> & wire)
{
  if (size < header_size || size > max_body || body[field::length] != size) {
    return 0;
  }

  std::size_t n = 0;
  uint8_t bcc = control::etx;
  wire[n++] = control::dle;
  wire[n++] = control::stx;

  for (std::size_t i = 0; i < size; ++i) {
    if (body[i] == control::dle) {
      wire[n++] = control::dle;
    }
    wire[n++] = body[i];
    bcc ^= body[i];
  }

  wire[n++] = control::dle;
  wire[n++] = control::etx;
  wire[n++] = bcc;
  return n;
}

int16_t signed16(const uint8_t * bytes)
{
  const uint16_t value = uint16_t(bytes[0]) | (uint16_t(bytes[1]) << 8);
  return static_cast<int16_t>(value <= 0x7fff ? int32_t(value) : int32_t(value) - 0x10000);
}

float float32(const uint8_t * bytes)
{
  static_assert(
    sizeof(float) == limit_value_size && std::numeric_limits<float>::is_iec559,
    "Sensor protocol requires "
    "IEEE-754 binary32");
  const uint32_t bits = uint32_t(bytes[0]) | (uint32_t(bytes[1]) << 8) |
    (uint32_t(bytes[2]) << 16) | (uint32_t(bytes[3]) << 24);
  float value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

/// Extract a bounded fixed-width string, trimming NUL and trailing space padding.
std::string fixedString(const uint8_t * bytes, std::size_t length)
{
  std::size_t end = 0;
  while (end < length && bytes[end] != 0) {
    ++end;
  }
  while (end > 0 && bytes[end - 1] == ' ') {
    --end;
  }
  return std::string(reinterpret_cast<const char *>(bytes), end);
}

Response validate(const uint8_t * body, std::size_t size, uint8_t expected)
{
  if (
    size < header_size || size > max_body || body[field::length] != size ||
    body[field::sensor] != sensor_id)
  {
    return Response::invalid;
  }
  if (body[field::command] != expected) {
    return Response::unrelated;
  }
  if (body[field::result] != result_code::success) {
    if (size != header_size) {
      return Response::invalid;
    }
    return body[field::result] == result_code::state_error ? Response::retry : Response::fatal;
  }

  switch (expected) {
    case command_code::get_info:
      return size == product_v113_size || size == product_v131_size ? Response::success
                                                                    : Response::invalid;
    case command_code::get_limits:
      return size == limits_size ? Response::success : Response::invalid;
    case command_code::get_filter:
      return size == filter_size ? Response::success : Response::invalid;
    case command_code::start:
      if (size == header_size) {
        return Response::success;
      }
      // v1.31 makes the final reserved byte unspecified, like bytes 16/17.
      return size == measurement_size ? Response::measurement : Response::invalid;
    case command_code::stop:
      return size == header_size ? Response::success : Response::invalid;
    default:
      return Response::invalid;
  }
}

/// Require printable text with a visible character and valid trailing padding.
bool validAscii(const uint8_t * bytes, std::size_t length)
{
  bool visible = false;
  bool padding = false;
  for (std::size_t i = 0; i < length; ++i) {
    const uint8_t c = bytes[i];
    if (c == 0) {
      padding = true;
      continue;
    }
    if (c < ' ' || c > '~' || (padding && c != ' ')) {
      return false;
    }
    if (c != ' ') {
      visible = true;
    }
  }
  return visible;
}

/// Require digits followed only by space or NUL padding within the serial field.
bool validSerial(const uint8_t * bytes)
{
  // The field is eight bytes wide. Real PFS firmware pads shorter serials with
  // spaces (e.g. seven digits + space); do not confuse width with digit count.
  std::size_t digits = 0;
  while (digits < field::serial_size && bytes[digits] >= '0' && bytes[digits] <= '9') {
    ++digits;
  }
  if (digits == 0) {
    return false;
  }
  for (std::size_t i = digits; i < field::serial_size; ++i) {
    if (bytes[i] != ' ' && bytes[i] != 0) {
      return false;
    }
  }
  return true;
}

/// Validate a supported product reply and all fixed-width text fields.
bool validProduct(const uint8_t * body, std::size_t size)
{
  return validate(body, size, command_code::get_info) == Response::success &&
    validAscii(body + field::model, field::model_size) && validSerial(body + field::serial) &&
    validAscii(body + field::firmware, field::firmware_size) &&
    (size == product_v113_size || validAscii(body + field::output_rate, field::output_rate_size));
}

bool decodeProduct(const uint8_t * body, std::size_t size, ProductInfo & info)
{
  if (!validProduct(body, size)) {
    return false;
  }

  ProductInfo candidate;
  candidate.protocol = size == product_v131_size ? Protocol::v131 : Protocol::v113;
  candidate.model = fixedString(body + field::model, field::model_size);
  candidate.serial = fixedString(body + field::serial, field::serial_size);
  candidate.firmware = fixedString(body + field::firmware, field::firmware_size);
  if (size == product_v131_size) {
    candidate.output_rate = fixedString(body + field::output_rate, field::output_rate_size);
  }

  info = candidate;
  return true;
}

bool conversionFactors(
  const uint8_t * body, std::size_t size, std::array<double, axis::count> & factors)
{
  if (validate(body, size, command_code::get_limits) != Response::success) {
    return false;
  }

  std::array<double, axis::count> candidate;
  for (std::size_t i = 0; i < candidate.size(); ++i) {
    const float limit = float32(body + field::payload + limit_value_size * i);
    if (!std::isfinite(limit) || limit <= 0) {
      return false;
    }
    candidate[i] = double(limit) * raw_to_fraction;
  }

  factors = candidate;  // Commit only when all six axes are valid.
  return true;
}

bool validFilter(const uint8_t * body, std::size_t size)
{
  if (
    validate(body, size, command_code::get_filter) != Response::success ||
    body[field::payload] > filter_code::hz200)
  {
    return false;
  }

  for (std::size_t i = field::payload + 1; i < size; ++i) {
    if (body[i] != 0) {
      return false;  // Reserved bytes must be zero.
    }
  }
  return true;
}

std::array<double, axis::count> decodeWrench(
  const uint8_t * body, const std::array<double, axis::count> & factors)
{
  std::array<double, axis::count> values;
  for (std::size_t i = 0; i < values.size(); ++i) {
    values[i] = signed16(body + field::payload + force_value_size * i) * factors[i];
  }
  return values;
}
}  // namespace leptrino_force_torque_ros_driver
