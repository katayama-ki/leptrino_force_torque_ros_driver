// SPDX-FileCopyrightText: Copyright katayama-ki
// SPDX-License-Identifier: MIT

#include "leptrino_force_torque_ros_driver/zero_wrench.hpp"

#include <cassert>
#include <future>
#include <thread>

namespace lw = leptrino_force_torque_ros_driver;

namespace
{
void sample(
  lw::ZeroWrench & zero, int value, uint8_t status = 0, lw::Deadline stamp = lw::Clock::now())
{
  std::array<uint8_t, lw::measurement_size> body{};
  std::array<double, lw::axis::count> factors{};
  for (std::size_t i = 0; i < lw::axis::count; ++i) {
    const uint16_t raw = static_cast<uint16_t>(value + i);
    body[lw::field::payload + 2 * i] = raw & 0xff;
    body[lw::field::payload + 2 * i + 1] = raw >> 8;
    factors[i] = i + 1;
  }
  body[lw::field::measurement_status] = status;
  zero.sample(body.data(), factors, stamp);
}

void offsetIs(lw::ZeroWrench & zero, double mean)
{
  std::array<double, lw::axis::count> values{};
  zero.subtract(values);
  for (std::size_t i = 0; i < lw::axis::count; ++i) {
    assert(values[i] == -(mean + i) * (i + 1));
  }
}
}  // namespace

int main()
{
  lw::ZeroWrench zero(2, 0.1);
  auto first = zero.start();
  auto waiter = std::async(std::launch::async, [&] { return zero.wait(first); });
  sample(zero, 100, 0, first->started - std::chrono::nanoseconds(1));
  sample(zero, 100, lw::status_bit::correction_error);
  sample(zero, 100, lw::status_bit::sensor_error);
  sample(zero, 100, lw::status_bit::overload);
  sample(zero, 10);
  lw::ZeroWrench::Result result;
  assert(!zero.poll(first, result));
  sample(zero, 13);
  assert(waiter.get().success);
  offsetIs(zero, 11.5);

  // New requests use raw values, and a late waiter retains its own request's result.
  auto second = zero.start();
  offsetIs(zero, 11.5);
  sample(zero, 20);
  sample(zero, 24);
  assert(zero.wait(second).success);
  assert(zero.wait(first).success);
  offsetIs(zero, 22);

  // Partial collection followed by silence must time out without changing the offset.
  auto timed_out = zero.start();
  sample(zero, 100);
  assert(!zero.wait(timed_out).success);
  sample(zero, 200);
  offsetIs(zero, 22);

  // The hardware loop can detect a silent startup timeout without a waiting callback.
  lw::ZeroWrench startup(2, 0.01);
  auto initial = startup.start();
  std::this_thread::sleep_until(initial->deadline);
  assert(startup.poll(initial, result) && !result.success);

  // Shutdown wakes an outstanding callback instead of waiting out a long timeout.
  lw::ZeroWrench stopping(2, 60);
  auto cancelled = stopping.start();
  auto blocked = std::async(std::launch::async, [&] { return stopping.wait(cancelled); });
  stopping.stop();
  assert(blocked.wait_for(std::chrono::seconds(1)) == std::future_status::ready);
  assert(!blocked.get().success);
}
