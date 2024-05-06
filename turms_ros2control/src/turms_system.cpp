// Copyright 2021 ros2_control Development Team
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

#include "turms_ros2control/turms_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace turms_ros2control
{
hardware_interface::return_type TurmsSystemHardware::configure(
  const hardware_interface::HardwareInfo & info)
{
  RCLCPP_INFO(rclcpp::get_logger("TurmsSystemHardware"), "Configuring hardware interface... please wait...");
  if (configure_default(info) != hardware_interface::return_type::OK)
  {
    return hardware_interface::return_type::ERROR;
  }
  // init comms config
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.encoder_ticks = std::stoi(info_.hardware_parameters["encoder_ticks"]);
  cfg_.servo_name = info_.hardware_parameters["servo_name"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.verbose = info_.hardware_parameters["verbose"]=="true";
  // init state variable
  left_wheel_.setup(cfg_.left_wheel_name, cfg_.encoder_ticks);
  right_wheel_.setup(cfg_.right_wheel_name, cfg_.encoder_ticks);
  servo_.setup(cfg_.servo_name);


  // check for proper ros2_control.xacro description
  /*for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // TurmsSystemHardware has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("TurmsSystemHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      //return hardware_interface::return_type::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("TurmsSystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      //return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("TurmsSystemHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      //return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("TurmsSystemHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      //return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("TurmsSystemHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      //return hardware_interface::return_type::ERROR;
    }
  }*/

  status_ = hardware_interface::status::CONFIGURED;

  RCLCPP_INFO(rclcpp::get_logger("TurmsSystemHardware"), "Configured");
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> TurmsSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
      left_wheel_.name, hardware_interface::HW_IF_POSITION, &left_wheel_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      left_wheel_.name, hardware_interface::HW_IF_VELOCITY, &left_wheel_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
      right_wheel_.name, hardware_interface::HW_IF_POSITION, &right_wheel_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      right_wheel_.name, hardware_interface::HW_IF_VELOCITY, &right_wheel_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
      servo_.name, hardware_interface::HW_IF_POSITION, &servo_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      servo_.name , "range", &servo_.ping));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> TurmsSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      left_wheel_.name, hardware_interface::HW_IF_VELOCITY, &left_wheel_.cmd_vel));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      right_wheel_.name, hardware_interface::HW_IF_VELOCITY, &right_wheel_.cmd_vel));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      servo_.name, hardware_interface::HW_IF_POSITION, &servo_.cmd_pos));
  

  return command_interfaces;
}

hardware_interface::return_type TurmsSystemHardware::start()
{
  RCLCPP_INFO(rclcpp::get_logger("TurmsSystemHardware"), "Starting ...please wait...");
  // starting arduino connection
  try{
    comms_.connect(cfg_.device, cfg_.baud_rate,cfg_.timeout_ms);
  }
  catch(const LibSerial::OpenFailed&){
    RCLCPP_ERROR(rclcpp::get_logger("TurmsSystemHardware"), "Cannot open serial port '%s'",cfg_.device.c_str());
    return hardware_interface::return_type::ERROR;
  }
  status_ = hardware_interface::status::STARTED;
  RCLCPP_INFO(rclcpp::get_logger("TurmsSystemHardware"), "System Successfully started!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TurmsSystemHardware::stop()
{
  RCLCPP_INFO(rclcpp::get_logger("TurmsSystemHardware"), "Stopping ...please wait...");
  // discconnecto from arduino
  comms_.disconnect();
  status_ = hardware_interface::status::STOPPED;
  RCLCPP_INFO(rclcpp::get_logger("TurmsSystemHardware"), "System successfully stopped!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TurmsSystemHardware::read()
{
  // read from serial port
  // encoders ticks
  comms_.read_encoder_values(left_wheel_.enc, right_wheel_.enc);
  left_wheel_.pos = left_wheel_.calc_enc_angle();
  right_wheel_.pos = right_wheel_.calc_enc_angle();

  // wheels speed
  comms_.read_wheels_speed(left_wheel_.vel, right_wheel_.vel);

  // servo position
  int servo_pos = 0;
  float ping = -1;
  comms_.read_lidar_values(0, servo_pos, ping);                         // read position [degrees] and ping [cm]
  servo_.pos = static_cast<double>(servo_pos)*M_PI/180.0 - M_PI/2;      // store position [rad]
  servo_.ping = static_cast<double>(ping/100.0);                        // store ping distance [m]


  if (cfg_.verbose){
    RCLCPP_INFO(rclcpp::get_logger("TurmsSystemHardware"), "Reading left wheel pos %d, right wheel pos %d, left wheel vel %.2f, right wheel vel %.2f,  servo pos %.0f, ping %.3f",
      left_wheel_.enc, right_wheel_.enc, left_wheel_.vel, right_wheel_.vel, servo_.pos*180.0/M_PI, servo_.ping);
  }

  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type turms_ros2control ::TurmsSystemHardware::write()
{
  int left_wheel_cmd_vel = static_cast<int>(left_wheel_.cmd_vel);
  int right_wheel_cmd_vel = static_cast<int>(right_wheel_.cmd_vel);
  int servo_cmd_pos = static_cast<int>(servo_.cmd_pos*180/M_PI + 90);
  
  // send commands over serial port
  if (cfg_.verbose){
  RCLCPP_INFO(rclcpp::get_logger("TurmsSystemHardware"), "Writing left wheel vel %d, right wheel vel %d, servo pos %d",
    left_wheel_cmd_vel, right_wheel_cmd_vel, servo_cmd_pos);
  }
  comms_.set_motor_values(left_wheel_cmd_vel, right_wheel_.cmd_vel);
  comms_.set_servo_value(0, servo_cmd_pos);

  return hardware_interface::return_type::OK;
}

}  // namespace turms_ros2control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  turms_ros2control::TurmsSystemHardware, 
  hardware_interface::SystemInterface
)
