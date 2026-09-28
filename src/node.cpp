// SPDX-License-Identifier: MIT
#include <geometry_msgs/WrenchStamped.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include <chrono>
#include <limits>
#include <stdexcept>
#include <string>

#include "leptrino_force_torque_ros_driver/publication.hpp"
#include "leptrino_force_torque_ros_driver/sensor.hpp"
#include "leptrino_force_torque_ros_driver/zero_wrench.hpp"

namespace lw = leptrino_force_torque_ros_driver;

namespace
{
/// Keep ROS callbacks off the hardware loop and cancel waits before joining callback threads.
class ZeroWrenchService
{
public:
  ZeroWrenchService(ros::NodeHandle & nh, lw::ZeroWrench & zero) : zero_(zero), spinner_(1)
  {
    service_ = nh.advertiseService("zero_wrench", &ZeroWrenchService::callback, this);
    spinner_.start();
  }

  ~ZeroWrenchService()
  {
    zero_.stop();
    spinner_.stop();
  }

private:
  bool callback(std_srvs::Trigger::Request &, std_srvs::Trigger::Response & response)
  {
    const auto request = zero_.start();
    const auto result = zero_.wait(request);
    response.success = result.success;
    response.message = result.message;
    return true;
  }

  lw::ZeroWrench & zero_;
  ros::ServiceServer service_;
  ros::AsyncSpinner spinner_;
};
}  // namespace

/// Publish each selected chunk's latest measurement and report startup/runtime/shutdown failures.
int main(int argc, char ** argv)
{
  ros::init(argc, argv, "leptrino");
  ros::NodeHandle nh("~");

  std::string port;
  if (!nh.getParam("serial_port", port) || port.empty()) {
    ROS_FATAL("~serial_port is required (e.g. /dev/ttyACM0 or /dev/serial/by-id/<device-id>)");
    return 1;
  }

  const std::string frame_id = nh.param<std::string>("frame_id", "leptrino");
  const int queue_size = nh.param("pub_queue_size", 1);
  const int retries = nh.param("command_retries", 2);
  const double pub_rate = nh.param("pub_rate", 0.0);
  const double timeout = nh.param("command_timeout", 1.0);
  const double warning_timeout = nh.param("warning_timeout", 1.0);
  const bool zero_on_start = nh.param("zero_wrench_on_start", false);
  const int zero_samples = nh.param("zero_wrench_samples", 100);
  const double zero_timeout = nh.param("zero_wrench_timeout", 0.5);

  if (
    frame_id.empty() || queue_size <= 0 || retries < 0 ||
    retries == std::numeric_limits<int>::max() || !lw::validTimeout(timeout) ||
    !lw::validRate(pub_rate) || !lw::validTimeout(warning_timeout) ||
    zero_samples <= 0 || !lw::validTimeout(zero_timeout))
  {
    ROS_FATAL("Invalid node parameters");
    return 1;
  }

  lw::ZeroWrench zero(zero_samples, zero_timeout);
  lw::ZeroWrench::Pending startup_zero;
  bool publication_enabled = false;
  const auto check_startup = [&] {
    if (startup_zero) {
      lw::ZeroWrench::Result result;
      if (zero.poll(startup_zero, result)) {
        if (!result.success) {
          throw std::runtime_error("Startup zero_wrench failed: " + result.message);
        }
        startup_zero.reset();
        publication_enabled = true;
        ROS_INFO("Startup zero_wrench succeeded");
      }
    }
  };

  lw::ChunkPublication publication(pub_rate);
  lw::StatusThrottle status;
  bool receiving = false;
  bool timeout_warned = false;
  lw::Deadline last_measurement{};
  lw::Deadline last_timeout_warning{};
  lw::Runtime runtime{
    [&] {
      check_startup();
      if (!ros::ok()) {
        return false;
      }
      if (receiving) {
        const auto now = lw::Clock::now();
        const double elapsed = std::chrono::duration<double>(now - last_measurement).count();
        if (
          elapsed >= warning_timeout &&
          (!timeout_warned || now - last_timeout_warning >= lw::notification_interval))
        {
          ROS_WARN(
            "No valid measurement frame for %.3f s (warning_timeout %.3f s)",
            elapsed, warning_timeout);
          last_timeout_warning = now;
          timeout_warned = true;
        }
      }
      return true;
    },
    [] { return ros::Time::now().toNSec(); },
    [](const std::string & message) { ROS_WARN("%s", message.c_str()); },
  };

  try {
    lw::Sensor sensor(port, timeout, retries, runtime);
    int result = 0;

    try {
      sensor.initialize();
      const auto & product = sensor.product();
      ROS_INFO(
        "Sensor: %s, serial %s, firmware %s; v%s response format", product.model.c_str(),
        product.serial.c_str(), product.firmware.c_str(),
        product.protocol == lw::Protocol::v131 ? "1.31" : "1.13");
      if (!product.output_rate.empty()) {
        ROS_INFO("Sensor output rate: %s", product.output_rate.c_str());
      }

      const auto & factors = sensor.factors();
      ROS_INFO(
        "Sensor limits Fx/Fy/Fz/Mx/My/Mz: %.9g %.9g %.9g %.9g %.9g %.9g",
        factors[lw::axis::fx] * lw::counts_at_rated_load,
        factors[lw::axis::fy] * lw::counts_at_rated_load,
        factors[lw::axis::fz] * lw::counts_at_rated_load,
        factors[lw::axis::mx] * lw::counts_at_rated_load,
        factors[lw::axis::my] * lw::counts_at_rated_load,
        factors[lw::axis::mz] * lw::counts_at_rated_load);
      if (sensor.filter() >= 0) {
        const char * names[] = {"OFF", "10 Hz", "100 Hz", "200 Hz"};
        ROS_INFO("Digital filter: %s", names[sensor.filter()]);
      }

      auto publisher = nh.advertise<geometry_msgs::WrenchStamped>("wrench", queue_size);
      std::unique_ptr<ZeroWrenchService> zero_service;
      bool publication_started = false;
      sensor.run(
        [&](const uint8_t * data, const lw::ReceiveStamp & stamp) {
          last_measurement = stamp.steady;
          timeout_warned = false;
          const auto warnings = status.warnings(lw::statusBits(data), stamp.steady);
          if (warnings & lw::status_bit::correction_error) {
            ROS_ERROR("Sensor status: correction data error (bit 0)");
          }
          if (warnings & lw::status_bit::sensor_error) {
            ROS_ERROR("Sensor status: sensor error (bit 1)");
          }
          if (warnings & lw::status_bit::overload) {
            ROS_WARN("Sensor status: rated range exceeded (bit 2)");
          }

          zero.sample(data, factors, stamp.steady);
          publication.update(data);
        },
        [&](const lw::ReceiveStamp & stamp) {
          const uint8_t * data = publication.finishChunk(stamp.steady);
          check_startup();
          if (publication_enabled && !zero_service) {
            zero_service.reset(new ZeroWrenchService(nh, zero));
          }
          if (!data || !publication_enabled) {
            return;
          }

          // Check the selected frame so an older healthy value cannot replace a faulty latest one.
          if (
            lw::statusBits(data) &
            (lw::status_bit::correction_error | lw::status_bit::sensor_error))
          {
            return;
          }

          auto values = lw::decodeWrench(data, factors);
          zero.subtract(values);
          geometry_msgs::WrenchStamped message;
          message.header.stamp.fromNSec(stamp.ros_nanoseconds);
          message.header.frame_id = frame_id;
          message.wrench.force.x = values[lw::axis::fx];
          message.wrench.force.y = values[lw::axis::fy];
          message.wrench.force.z = values[lw::axis::fz];
          message.wrench.torque.x = values[lw::axis::mx];
          message.wrench.torque.y = values[lw::axis::my];
          message.wrench.torque.z = values[lw::axis::mz];
          publisher.publish(message);

          if (!publication_started) {
            publication_started = true;
            if (pub_rate > 0) {
              ROS_INFO(
                "Publishing started on %s: target %.9g Hz", publisher.getTopic().c_str(), pub_rate);
            } else if (!product.output_rate.empty()) {
              ROS_INFO(
                "Publishing started on %s: latest per receive chunk (sensor rate %s Hz)",
                publisher.getTopic().c_str(), product.output_rate.c_str());
            } else {
              ROS_INFO(
                "Publishing started on %s: latest per receive chunk (sensor rate unavailable)",
                publisher.getTopic().c_str());
            }
          }
        },
        [&] {
          last_measurement = lw::Clock::now();
          receiving = true;
          if (zero_on_start) {
            startup_zero = zero.start();
          } else {
            publication_enabled = true;
          }
        });
    } catch (const std::exception & error) {
      ROS_FATAL("%s", error.what());
      result = 1;
    }

    try {
      sensor.stop();
    } catch (const std::exception & error) {
      ROS_ERROR("Shutdown: %s", error.what());
      result = 1;
    }
    if (!sensor.close()) {
      ROS_ERROR("Serial close failed");
      result = 1;
    }

    return result;
  } catch (const std::exception & error) {
    ROS_FATAL("%s", error.what());
    return 1;
  }
}
