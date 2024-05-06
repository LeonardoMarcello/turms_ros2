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

#include "turms_ros2control/lidar_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

#include "hardware_interface/loaned_command_interface.hpp"

namespace lidar_controllers
{
using hardware_interface::LoanedCommandInterface;

LidarController::LidarController()
: controller_interface::ControllerInterface(),
  rt_command_ptr_(nullptr),
  joints_command_subscriber_(nullptr),
  ranges_measure_publisher_(nullptr)
{
}

controller_interface::return_type LidarController::init(
  const std::string & controller_name)
{
  auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::OK)
  {
    return ret;
  }

  try
  {
    // with the lifecycle node being initialized, we can declare parameters
    auto_declare<std::string>("frame_id", "");
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    auto_declare<std::string>("interface_name", "");

    auto_declare<double>("publish_rate", lidar_params_.publish_rate_);
    auto_declare<double>("angle_min", lidar_params_.angle_min);
    auto_declare<double>("angle_max", lidar_params_.angle_max);
    auto_declare<double>("angle_increment", lidar_params_.angle_increment);
    auto_declare<double>("time_increment", lidar_params_.time_increment);
    auto_declare<double>("scan_time", lidar_params_.scan_time);
    auto_declare<double>("range_min", lidar_params_.range_min);
    auto_declare<double>("range_max", lidar_params_.range_max);

  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

CallbackReturn LidarController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // parsing parameters
  frame_id_ = node_->get_parameter("frame_id").as_string();
  joint_names_ = node_->get_parameter("joints").as_string_array();
  interface_name_ = node_->get_parameter("interface_name").as_string();
  lidar_params_.publish_rate_ = node_->get_parameter("publish_rate").as_double();
  lidar_params_.angle_min = node_->get_parameter("angle_min").as_double();
  lidar_params_.angle_max = node_->get_parameter("angle_max").as_double();
  lidar_params_.angle_increment = node_->get_parameter("angle_increment").as_double();
  lidar_params_.time_increment = node_->get_parameter("time_increment").as_double();
  lidar_params_.scan_time = node_->get_parameter("scan_time").as_double();
  lidar_params_.range_min = node_->get_parameter("range_min").as_double();
  lidar_params_.range_max = node_->get_parameter("range_max").as_double();

  if (joint_names_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
    return CallbackReturn::ERROR;
  }

  // Specialized, child controllers set interfaces before calling configure function.
  if (interface_name_.empty())
  {
    interface_name_ = node_->get_parameter("interface_name").as_string();
  }

  if (interface_name_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'interface_name' parameter was empty");
    return CallbackReturn::ERROR;
  }
  
  // commands subscribers
  joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
    "~/commands", rclcpp::SystemDefaultsQoS(),
    [this](const CmdType::SharedPtr msg) { rt_command_ptr_.writeFromNonRT(msg); });
  
  // sensor publisher
  ranges_measure_publisher_ = get_node()->create_publisher<MeasType>(
    "/scan", rclcpp::SystemDefaultsQoS());
  // limit the publication on the topics /odom and /tf
  publish_period_ = rclcpp::Duration::from_seconds(1.0 / lidar_params_.publish_rate_);
  prev_ranges_measure_time_ = node_->get_clock()->now();

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
LidarController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint : joint_names_)
  {
    command_interfaces_config.names.push_back(joint + "/" + interface_name_);
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
LidarController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint : joint_names_)
  {
    state_interfaces_config.names.push_back(joint + "/" + interface_name_);
    state_interfaces_config.names.push_back(joint + "/range" );
  }

  return state_interfaces_config;
}

// Fill ordered_interfaces with references to the matching interfaces
// in the same order as in joint_names
template <typename T>
bool get_ordered_interfaces(
  std::vector<T> & unordered_interfaces, const std::vector<std::string> & joint_names,
  const std::string & interface_type, std::vector<std::reference_wrapper<T>> & ordered_interfaces)
{
  for (const auto & joint_name : joint_names)
  {
    for (auto & command_interface : unordered_interfaces)
    {
      if (
        (command_interface.get_name() == joint_name) &&
        (command_interface.get_interface_name() == interface_type))
      {
        ordered_interfaces.push_back(std::ref(command_interface));
      }
    }
  }

  return joint_names.size() == ordered_interfaces.size();
}

CallbackReturn LidarController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  //  check if we have all resources defined in the "points" parameter
  //  also verify that we *only* have the resources defined in the "points" parameter
  std::vector<std::reference_wrapper<LoanedCommandInterface>> ordered_interfaces;
  if (
    !get_ordered_interfaces(
      command_interfaces_, joint_names_, interface_name_, ordered_interfaces) ||
    command_interfaces_.size() != ordered_interfaces.size())
  {
    RCLCPP_ERROR(
      node_->get_logger(), "Expected %zu position command interfaces, got %zu", joint_names_.size(),
      ordered_interfaces.size());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // reset command buffer if a command came through callback when controller was inactive
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);

  return CallbackReturn::SUCCESS;
}

CallbackReturn LidarController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // reset command buffer
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type LidarController::update()
{
  // read state interfaces
  std::vector<float> ranges(state_interfaces_.size());
  double range = -1.0;
  for (auto index = 1ul; index < state_interfaces_.size(); index+=2) {
    range = state_interfaces_[index].get_value();
    ranges[index-1] = range;
    ranges[index] = range;
    if (std::isnan(range)) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Lidar measured range is invalid for index [%zu]", index);
      return controller_interface::return_type::ERROR;
    }
  }
  auto current_time = node_->get_clock()->now();
  if (prev_ranges_measure_time_ + publish_period_ < current_time){
    // publish measures
    /*//Laserscan ver
    auto range_msg = sensor_msgs::msg::LaserScan();
    range_msg.header.frame_id = frame_id_;
    range_msg.header.stamp = current_time;
    range_msg.angle_min = lidar_params_.angle_min;
    range_msg.angle_max = lidar_params_.angle_max;
    range_msg.angle_increment = lidar_params_.angle_increment;
    range_msg.time_increment = lidar_params_.time_increment;
    range_msg.scan_time = lidar_params_.scan_time;
    range_msg.range_min = lidar_params_.range_min;
    range_msg.range_max = lidar_params_.range_max;
    range_msg.ranges = ranges;*/
    // Range ver
    auto range_msg = sensor_msgs::msg::Range();
    range_msg.header.frame_id = frame_id_;
    range_msg.header.stamp = current_time;
    range_msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    range_msg.min_range = lidar_params_.range_min;
    range_msg.max_range = lidar_params_.range_max;
    range_msg.range = range;

    ranges_measure_publisher_->publish(range_msg);

    prev_ranges_measure_time_ = current_time;
  }

  // read command
  auto joint_commands = rt_command_ptr_.readFromRT();
  // no command received yet
  if (!joint_commands || !(*joint_commands))
  {
    return controller_interface::return_type::OK;
  }
  // incoherent command size
  if ((*joint_commands)->data.size() != command_interfaces_.size())
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *node_->get_clock(), 1000,
      "command size (%zu) does not match number of interfaces (%zu)",
      (*joint_commands)->data.size(), command_interfaces_.size());
    return controller_interface::return_type::ERROR;
  }
  // set command interfaces
  for (auto index = 0ul; index < command_interfaces_.size(); ++index)
  {
    command_interfaces_[index].set_value((*joint_commands)->data[index]);
  }
  return controller_interface::return_type::OK;
}

}  // namespace lidar_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  lidar_controllers::LidarController, controller_interface::ControllerInterface)