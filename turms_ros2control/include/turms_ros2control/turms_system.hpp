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

#ifndef TURMS_HARDWARE__DIFFBOT_SYSTEM_HPP_
#define TURMS_HARDWARE__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/logger.hpp"

#include "hardware/arduino_comms.hpp"

#include "turms_ros2control/visibility_control.h"
#include "turms_ros2control/dcmotor.hpp"
#include "turms_ros2control/servomotor.hpp"

using namespace arduino_comms;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


namespace turms_ros2control
{
class TurmsSystemHardware : public hardware_interface::SystemInterface
{
struct Config{
    std::string device = "";
    std::string left_wheel_name = "";
    std::string right_wheel_name = "";
    int encoder_ticks = 0;
    std::string servo_name = "";
    float loop_rate = 0.0;
    int baud_rate = 0;
    int timeout_ms = 0;
    bool verbose = 0;
};
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(TurmsSystemHardware);

  TURMS_HARDWARE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  TURMS_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  TURMS_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  TURMS_HARDWARE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & State) override;

  TURMS_HARDWARE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & State) override;

  TURMS_HARDWARE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time & time,const rclcpp::Duration & period) override;

  TURMS_HARDWARE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  // instance of Arduino Serial
  ArduinoComms comms_;
  Config cfg_;
  
  // Instance of status variable
  Wheel left_wheel_;
  Wheel right_wheel_;
  Servo servo_;
};

}  // namespace turms_ros2control

#endif  // TURMS_HARDWARE__DIFFBOT_SYSTEM_HPP_
