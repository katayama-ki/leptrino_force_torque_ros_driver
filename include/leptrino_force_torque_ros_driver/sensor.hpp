// SPDX-License-Identifier: MIT
#ifndef LEPTRINO_WRENCH_SENSOR_HPP_
#define LEPTRINO_WRENCH_SENSOR_HPP_

#include <functional>

#include "leptrino_force_torque_ros_driver/serial_port.hpp"

namespace leptrino_force_torque_ros_driver
{
/// Runtime hooks keep ROS out of the sensor; serial I/O and deadlines are concrete.
struct Runtime
{
  std::function<bool()> running;
  std::function<uint64_t()> timestamp;
  std::function<void(const std::string &)> warn;
};

/// Check that a positive finite timeout can be represented safely by the monotonic clock.
bool validTimeout(double seconds);

/// Manage validated sensor commands and streaming through synchronous callbacks.
/// Not thread-safe or reentrant. Cancel via Runtime::running and let run()
/// return before calling stop(); do not call lifecycle methods from callbacks.
class Sensor
{
public:
  /// Called for every valid measurement; data is borrowed only for the callback duration.
  using Measurement = std::function<void(const uint8_t *, const ReceiveStamp &)>;
  /// Called after each successful read is parsed, even if no measurement completed.
  using ChunkDone = std::function<void(const ReceiveStamp &)>;

  /// Open the device; timeout is per command attempt and retries exclude the first attempt.
  Sensor(const std::string & port, double timeout, int retries, Runtime runtime);
  /// Attempt STOP without throwing; the owned port is then closed.
  ~Sensor();

  /// Confirm STOP, then obtain validated product data, factors and any supported filter setting.
  void initialize();
  /// Confirm START and process reads until runtime cancellation; command/I/O failures throw.
  /// Requires successful initialize(); chunk_done also follows reads containing START responses.
  /// started runs immediately after START is confirmed, before dispatching measurements.
  /// May be called again after successful stop(), with runtime cancellation cleared.
  void run(
    const Measurement & measurement, const ChunkDone & chunk_done,
    const std::function<void()> & started = {});
  /// Confirm STOP after any START attempt; failure throws.
  void stop();

  /// Close the port; call stop() first to confirm streaming has ended.
  bool close()
  {
    return port_.close();
  }

  /// Product fields populated by successful initialization.
  const ProductInfo & product() const
  {
    return product_;
  }

  /// Validated per-axis conversion factors populated by successful initialization.
  const std::array<double, axis::count> & factors() const
  {
    return factors_;
  }

  /// -1: not queried for v1.31.
  int filter() const
  {
    return filter_;
  }

private:
  using Frame = std::function<void(Event, const uint8_t *, std::size_t, const ReceiveStamp &)>;

  /// Parse one available chunk, then signal completion; throw on serial failure.
  bool receive(const Frame & frame, Deadline deadline);
  /// Bound transmission and matching-response validation by one deadline per attempt.
  void command(uint8_t code, int retries);
  /// Validate and commit command payloads after header/result validation.
  bool accept(uint8_t code, const uint8_t * data, std::size_t size);

  bool running() const
  {
    return ignore_runtime_stop_ || runtime_.running();
  }

  SerialPort port_;
  Parser parser_;
  Clock::duration timeout_;
  int retries_;
  int filter_ = -1;
  Runtime runtime_;
  ProductInfo product_;
  std::array<double, axis::count> factors_{};

  bool initialized_ = false;
  bool start_sent_ = false;
  bool streaming_ = false;
  bool ignore_runtime_stop_ = false;

  Measurement measurement_;
  ChunkDone chunk_done_;
  std::function<void()> started_;
};
}  // namespace leptrino_force_torque_ros_driver

#endif  // LEPTRINO_WRENCH_SENSOR_HPP_
