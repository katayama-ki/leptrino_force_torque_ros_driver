// SPDX-License-Identifier: MIT
#ifndef LEPTRINO_WRENCH_SERIAL_PORT_HPP_
#define LEPTRINO_WRENCH_SERIAL_PORT_HPP_

#include <sys/types.h>

#include <string>

#include "leptrino_force_torque_ros_driver/protocol.hpp"

namespace leptrino_force_torque_ros_driver
{
/// Own a nonblocking 460800-baud raw serial port and close it on destruction.
class SerialPort
{
public:
  /// Open and configure the device; throw on failure.
  explicit SerialPort(const std::string & path);
  ~SerialPort();

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  /// Poll in attempts of up to 10 ms using the deadline; return events, or zero if not ready.
  /// Throw on a closed port or poll failure; callers handle returned error/hangup flags.
  short wait(short events, Deadline deadline);
  /// Read available bytes without filling the buffer; preserve POSIX count/errno semantics.
  ssize_t read(uint8_t * bytes, std::size_t size);
  /// Attempt a nonblocking write; callers handle short writes and POSIX errors.
  ssize_t write(const uint8_t * bytes, std::size_t size);
  /// Close once; return true if already closed or the close succeeds.
  bool close();

private:
  int fd_ = -1;
};
}  // namespace leptrino_force_torque_ros_driver

#endif  // LEPTRINO_WRENCH_SERIAL_PORT_HPP_
