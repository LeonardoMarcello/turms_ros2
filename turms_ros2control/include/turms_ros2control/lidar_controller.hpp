// Copyright 2020 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TURMS_CONTROLLERS__LIDAR_CONTROLLER_HPP_
#define TURMS_CONTROLLERS__LIDAR_CONTROLLER_HPP_

#include <string>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "turms_ros2control/visibility_control.h"

namespace lidar_controllers{

using CmdType = std_msgs::msg::Float64MultiArray;
//using MeasType = sensor_msgs::msg::LaserScan;
using MeasType = sensor_msgs::msg::Range;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * \brief Forward command controller for a set of joints.
 *
 * This class forwards the command signal down to a set of joints
 * on the specified interface. It broadcast measur from lidar
 *
 * \param frame_id Name of the frame id.
 * \param joints Names of the joints to control.
 * \param interface_name Name of the interface to command.
 *
 * Subscribes to:
 * - \b commands (std_msgs::msg::Float64MultiArray) : The commands to apply.
 * Publish to:
 * - \b sensor (sensor_msgs::msg::LaserScan) : The range measured.
 */
class LidarController : public controller_interface::ControllerInterface{

public:
  TURMS_HARDWARE_PUBLIC
  LidarController();

  TURMS_HARDWARE_PUBLIC
  controller_interface::return_type init(const std::string & controller_name) override;

  TURMS_HARDWARE_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  TURMS_HARDWARE_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  TURMS_HARDWARE_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  TURMS_HARDWARE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  TURMS_HARDWARE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  TURMS_HARDWARE_PUBLIC
  controller_interface::return_type update() override;

protected:
  std::string frame_id_;
  std::vector<std::string> joint_names_;
  std::string interface_name_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_command_ptr_;
  rclcpp::Subscription<CmdType>::SharedPtr joints_command_subscriber_;
  rclcpp::Publisher<MeasType>::SharedPtr ranges_measure_publisher_;
  // publish rate limiter
  rclcpp::Duration publish_period_{0, 0};
  rclcpp::Time prev_ranges_measure_time_;
  
  std::string logger_name_;

  struct LidarHandle {
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> position;
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> range;
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity;
  };
  std::vector<LidarHandle> lidar_handles_;

  struct LidarParams  {
    double publish_rate_ = 50.0;
    double angle_min;
    double angle_max;
    double angle_increment;
    double time_increment;
    double scan_time;
    double range_min;
    double range_max;
  } lidar_params_;
};

}   // namespace lidar_controllers

#endif  // TURMS_CONTROLLERS__LIDAR_CONTROLLER_HPP_