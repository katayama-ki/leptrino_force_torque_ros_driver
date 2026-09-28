// SPDX-License-Identifier: MIT
#ifndef LEPTRINO_WRENCH_PUBLICATION_HPP_
#define LEPTRINO_WRENCH_PUBLICATION_HPP_

#include "leptrino_force_torque_ros_driver/protocol.hpp"

namespace leptrino_force_torque_ros_driver
{
/// Accept zero or a positive finite rate representable by the monotonic clock.
inline bool validRate(double rate)
{
  if (!std::isfinite(rate) || rate < 0) {
    return false;
  }
  if (rate == 0) {
    return true;
  }

  const long double ticks =
    static_cast<long double>(Clock::period::den) / Clock::period::num / rate;
  return ticks >= 1 && ticks < static_cast<long double>(Clock::duration::max().count()) / 4;
}

/// Select receive times at an average rate, skipping elapsed slots without catch-up.
class PublicationSchedule
{
public:
  /// @param rate Valid publication rate in Hz; zero disables rate limiting.
  explicit PublicationSchedule(double rate)
  : all_(rate == 0),
    period_ticks_(
      all_ ? 0 : static_cast<long double>(Clock::period::den) / Clock::period::num / rate)
  {
  }

  /// Select a completed chunk at its monotonic receive time and advance past elapsed slots.
  bool select(Deadline received)
  {
    if (all_) {
      return true;
    }

    if (!started_) {
      anchor_ = received;
      started_ = true;
    }

    const long double elapsed = (received - anchor_).count();
    if (elapsed < next_slot_ * period_ticks_) {
      return false;
    }

    next_slot_ = static_cast<uint64_t>(std::floor(elapsed / period_ticks_)) + 1;
    return true;
  }

private:
  bool all_;
  bool started_ = false;
  long double period_ticks_;
  Deadline anchor_{};
  uint64_t next_slot_ = 0;
};

/// Retain the latest valid measurement for one publication decision per receive chunk.
class ChunkPublication
{
public:
  /// @param rate Valid publication rate in Hz; zero selects every measurement-bearing chunk.
  explicit ChunkPublication(double rate) : schedule_(rate)
  {
  }

  /// Copy a validated measurement before the parser reuses its frame storage.
  void update(const uint8_t * data)
  {
    std::memcpy(latest_.data(), data, latest_.size());
    pending_ = true;
  }

  /// Call once after parsing a read; consume its candidate, including when rate-limited.
  /// @return Selected body, valid until the next update(), or nullptr if nothing is due.
  const uint8_t * finishChunk(Deadline received)
  {
    const bool pending = pending_;
    pending_ = false;
    return pending && schedule_.select(received) ? latest_.data() : nullptr;
  }

private:
  PublicationSchedule schedule_;
  std::array<uint8_t, measurement_size> latest_{};
  bool pending_ = false;
};

/// Shared interval for repeated sensor-status and missing-measurement notifications.
constexpr std::chrono::seconds notification_interval{5};

/// Independent per-bit throttling remains live even when ROS time is paused.
class StatusThrottle
{
public:
  /// Return error bits due for a warning, at most once every five seconds for each bit.
  uint8_t warnings(uint8_t status, Deadline received)
  {
    uint8_t result = 0;
    for (std::size_t bit = 0; bit < last_.size(); ++bit) {
      const uint8_t mask = 1u << bit;
      if ((status & mask) && (!(seen_ & mask) || received - last_[bit] >= notification_interval))
      {
        result |= mask;
        seen_ |= mask;
        last_[bit] = received;
      }
    }
    return result;
  }

private:
  std::array<Deadline, status_bit::error_count> last_{};
  uint8_t seen_ = 0;
};
}  // namespace leptrino_force_torque_ros_driver

#endif  // LEPTRINO_WRENCH_PUBLICATION_HPP_
