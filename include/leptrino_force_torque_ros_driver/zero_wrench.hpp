// SPDX-License-Identifier: MIT
#ifndef LEPTRINO_WRENCH_ZERO_WRENCH_HPP_
#define LEPTRINO_WRENCH_ZERO_WRENCH_HPP_

#include <condition_variable>
#include <memory>
#include <mutex>

#include "leptrino_force_torque_ros_driver/protocol.hpp"

namespace leptrino_force_torque_ros_driver
{
/// Share a bounded zeroing request between service callbacks and the receive loop.
class ZeroWrench
{
public:
  struct Result
  {
    bool success;
    std::string message;
  };

  struct Request
  {
    explicit Request(Clock::duration timeout) : started(Clock::now()), deadline(started + timeout)
    {
    }

    const Deadline started;
    const Deadline deadline;

  private:
    friend class ZeroWrench;
    std::array<double, axis::count> sum{};
    int count = 0;
    bool done = false;
    Result result{false, ""};
  };
  using Pending = std::shared_ptr<Request>;

  /// Parameters must already be validated: samples > 0 and validTimeout(timeout).
  ZeroWrench(int samples, double timeout)
  : samples_(samples),
    timeout_(std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(timeout)))
  {
  }

  /// Start a fresh collection; the caller must serialize requests until completion.
  Pending start()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    current_ = std::make_shared<Request>(timeout_);
    if (stopped_) {
      finish(current_, false, "Zeroing cancelled: sensor loop stopped.");
    }
    return current_;
  }

  /// Process every validated measurement, before publication rate selection.
  void sample(
    const uint8_t * data, const std::array<double, axis::count> & factors, Deadline received)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!current_ || current_->done) {
      return;
    }
    expire(current_);
    if (current_->done || received < current_->started || statusBits(data)) {
      return;
    }

    const auto values = decodeWrench(data, factors);
    for (std::size_t i = 0; i < axis::count; ++i) {
      current_->sum[i] += values[i];
    }
    if (++current_->count == samples_) {
      for (std::size_t i = 0; i < axis::count; ++i) {
        offset_[i] = current_->sum[i] / samples_;
      }
      finish(current_, true, "Wrench zeroed successfully.");
    }
  }

  /// Wait only on a callback thread; the receive loop never waits for samples.
  Result wait(const Pending & request)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    changed_.wait_until(lock, request->deadline, [&] { return request->done; });
    expire(request);
    return request->result;
  }

  /// Nonblocking completion check, also detecting a timeout when no data arrives.
  bool poll(const Pending & request, Result & result)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    expire(request);
    result = request->result;
    return request->done;
  }

  void subtract(std::array<double, axis::count> & values)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (std::size_t i = 0; i < axis::count; ++i) {
      values[i] -= offset_[i];
    }
  }

  /// Wake service callbacks before stopping their spinner, including during exception unwinding.
  void stop()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopped_ = true;
    if (current_ && !current_->done) {
      finish(current_, false, "Zeroing cancelled: sensor loop stopped.");
    }
  }

private:
  void finish(const Pending & request, bool success, const char * message)
  {
    request->result = {success, message};
    request->done = true;
    changed_.notify_all();
  }

  void expire(const Pending & request)
  {
    if (!request->done && Clock::now() >= request->deadline) {
      finish(request, false, "Zeroing timed out.");
    }
  }

  const int samples_;
  const Clock::duration timeout_;
  std::mutex mutex_;
  std::condition_variable changed_;
  Pending current_;
  std::array<double, axis::count> offset_{};
  bool stopped_ = false;
};
}  // namespace leptrino_force_torque_ros_driver

#endif  // LEPTRINO_WRENCH_ZERO_WRENCH_HPP_
