// SPDX-License-Identifier: MIT
#ifndef LEPTRINO_WRENCH_OUTPUT_ROTATION_HPP_
#define LEPTRINO_WRENCH_OUTPUT_ROTATION_HPP_

#include <array>
#include <cmath>

#include "leptrino_force_torque_ros_driver/protocol.hpp"

namespace leptrino_force_torque_ros_driver
{
/// Rotate force and torque about the sensor origin using fixed-axis XYZ RPY in radians.
class OutputRotation
{
public:
  /// Finite angles in the same convention as tf::Matrix3x3::setRPY().
  OutputRotation(double roll, double pitch, double yaw)
  : identity_(roll == 0 && pitch == 0 && yaw == 0)
  {
    const double cr = std::cos(roll), sr = std::sin(roll);
    const double cp = std::cos(pitch), sp = std::sin(pitch);
    const double cy = std::cos(yaw), sy = std::sin(yaw);
    // Rz(yaw) * Ry(pitch) * Rx(roll), mapping sensor components to output components.
    matrix_ = {{{cp * cy, sp * sr * cy - cr * sy, sp * cr * cy + sr * sy},
                {cp * sy, sp * sr * sy + cr * cy, sp * cr * sy - sr * cy},
                {-sp, cp * sr, cp * cr}}};
  }

  void apply(std::array<double, axis::count> & values) const
  {
    if (identity_) {
      return;
    }
    const double fx = values[axis::fx], fy = values[axis::fy], fz = values[axis::fz];
    const double mx = values[axis::mx], my = values[axis::my], mz = values[axis::mz];
    values[axis::fx] = matrix_[0][0] * fx + matrix_[0][1] * fy + matrix_[0][2] * fz;
    values[axis::fy] = matrix_[1][0] * fx + matrix_[1][1] * fy + matrix_[1][2] * fz;
    values[axis::fz] = matrix_[2][0] * fx + matrix_[2][1] * fy + matrix_[2][2] * fz;
    values[axis::mx] = matrix_[0][0] * mx + matrix_[0][1] * my + matrix_[0][2] * mz;
    values[axis::my] = matrix_[1][0] * mx + matrix_[1][1] * my + matrix_[1][2] * mz;
    values[axis::mz] = matrix_[2][0] * mx + matrix_[2][1] * my + matrix_[2][2] * mz;
  }

private:
  bool identity_;
  std::array<std::array<double, 3>, 3> matrix_;
};
}  // namespace leptrino_force_torque_ros_driver

#endif  // LEPTRINO_WRENCH_OUTPUT_ROTATION_HPP_
