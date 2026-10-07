// SPDX-License-Identifier: MIT
#include "leptrino_force_torque_ros_driver/output_rotation.hpp"

#include <cassert>

using namespace leptrino_force_torque_ros_driver;

namespace
{
void check(const std::array<double, 3> & rpy, const std::array<double, axis::count> & expected)
{
  std::array<double, axis::count> values{{1, 2, 3, 4, 5, 6}};
  OutputRotation(rpy[0], rpy[1], rpy[2]).apply(values);
  for (std::size_t i = 0; i < values.size(); ++i) {
    assert(std::abs(values[i] - expected[i]) < 1e-12);
  }
}

void checkMixedAngles()
{
  const std::array<double, 3> rpy{{0.3, -0.7, 1.1}};
  std::array<double, axis::count> expected{{1, 2, 3, 4, 5, 6}};
  // Independently apply X, then Y, then Z rotations to test order and direction.
  for (std::size_t base : {std::size_t(axis::fx), std::size_t(axis::mx)}) {
    for (std::size_t axis = 0; axis < 3; ++axis) {
      const std::size_t a = base + (axis + 1) % 3;
      const std::size_t b = base + (axis + 2) % 3;
      const double va = expected[a], vb = expected[b];
      expected[a] = std::cos(rpy[axis]) * va - std::sin(rpy[axis]) * vb;
      expected[b] = std::sin(rpy[axis]) * va + std::cos(rpy[axis]) * vb;
    }
  }
  check(rpy, expected);

  // Rotation must not introduce force-dependent torque (the origin stays fixed).
  std::array<double, axis::count> force_only{{1, 2, 3, 0, 0, 0}};
  OutputRotation(rpy[0], rpy[1], rpy[2]).apply(force_only);
  assert(force_only[axis::mx] == 0 && force_only[axis::my] == 0 && force_only[axis::mz] == 0);
}
}  // namespace

int main()
{
  check({{0, 0, 0}}, {{1, 2, 3, 4, 5, 6}});
  const double half_pi = std::acos(-1.0) / 2;
  check({{half_pi, 0, 0}}, {{1, -3, 2, 4, -6, 5}});
  check({{0, half_pi, 0}}, {{3, 2, -1, 6, 5, -4}});
  check({{0, 0, half_pi}}, {{-2, 1, 3, -5, 4, 6}});
  check({{0, 0, -half_pi}}, {{2, -1, 3, 5, -4, 6}});
  check({{half_pi, 0, half_pi}}, {{3, 1, 2, 6, 4, 5}});
  checkMixedAngles();
}
