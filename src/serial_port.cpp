// SPDX-License-Identifier: MIT
#include "leptrino_force_torque_ros_driver/serial_port.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <stdexcept>

namespace leptrino_force_torque_ros_driver
{
namespace
{
constexpr double poll_interval_ms = 10.0;

std::runtime_error ioError(const char * operation)
{
  return std::runtime_error(std::string(operation) + ": " + std::strerror(errno));
}
}  // namespace

SerialPort::SerialPort(const std::string & path)
{
  fd_ = ::open(path.c_str(), O_RDWR | O_NONBLOCK | O_NOCTTY | O_CLOEXEC);
  if (fd_ < 0) {
    throw ioError("open serial port");
  }

  termios settings;
  if (tcgetattr(fd_, &settings) != 0) {
    const auto error = ioError("get serial settings");
    close();
    throw error;
  }

  cfmakeraw(&settings);
  settings.c_cflag = CS8 | CREAD | CLOCAL;
  settings.c_cc[VMIN] = 0;
  settings.c_cc[VTIME] = 0;
  if (
    cfsetispeed(&settings, B460800) != 0 || cfsetospeed(&settings, B460800) != 0 ||
    tcsetattr(fd_, TCSANOW, &settings) != 0)
  {
    const auto error = ioError("set serial settings");
    close();
    throw error;
  }
}

SerialPort::~SerialPort()
{
  close();
}

bool SerialPort::close()
{
  const int fd = fd_;
  fd_ = -1;
  return fd < 0 || ::close(fd) == 0;
}

short SerialPort::wait(short events, Deadline deadline)
{
  if (fd_ < 0) {
    throw std::runtime_error("serial port is closed");
  }

  for (;;) {
    const auto now = Clock::now();
    if (now >= deadline) {
      return 0;
    }

    const double remaining = std::chrono::duration<double, std::milli>(deadline - now).count();
    const int timeout = static_cast<int>(std::ceil(std::min(poll_interval_ms, remaining)));
    pollfd descriptor{fd_, events, 0};
    const int result = ::poll(&descriptor, 1, timeout);
    if (result < 0 && errno == EINTR) {
      continue;
    }
    if (result < 0) {
      throw ioError("poll serial port");
    }
    return descriptor.revents;
  }
}

ssize_t SerialPort::read(uint8_t * bytes, std::size_t size)
{
  return ::read(fd_, bytes, size);
}

ssize_t SerialPort::write(const uint8_t * bytes, std::size_t size)
{
  return ::write(fd_, bytes, size);
}
}  // namespace leptrino_force_torque_ros_driver
