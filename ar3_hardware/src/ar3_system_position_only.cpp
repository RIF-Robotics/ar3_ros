// Copyright 2020 ros2_control Development Team
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

#include "ar3_hardware/ar3_system_position_only.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ar3_hardware
{
CallbackReturn AR3SystemPositionOnlyHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  serial_device_ = info_.hardware_parameters["serial_device"];
  serial_baudrate_ = std::stod(info_.hardware_parameters["serial_baudrate"]);
  firmware_version_ = info_.hardware_parameters["firmware_version"];

  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // AR3SystemPositionOnly has exactly one state and command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("AR3SystemPositionOnlyHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("AR3SystemPositionOnlyHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("AR3SystemPositionOnlyHardware"),
        "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("AR3SystemPositionOnlyHardware"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn AR3SystemPositionOnlyHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (not comm_.init(serial_device_, serial_baudrate_, firmware_version_))
  {
    RCLCPP_ERROR(rclcpp::get_logger("AR3SystemPositionOnlyHardware"),
                 "Failed to initialize serial comm with the following parameters:\nserial_device=%s\nserial_baudrate=%d\nfirmware_version=%s",
                 serial_device_.c_str(), serial_baudrate_, firmware_version_.c_str());
    return CallbackReturn::FAILURE;
  }

  // Read the current joint positions and initialize states and commands
  comm_.get_joint_positions(hw_states_);
  hw_commands_ = hw_states_;

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
AR3SystemPositionOnlyHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
AR3SystemPositionOnlyHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

CallbackReturn AR3SystemPositionOnlyHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // command and state should be equal when starting
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_commands_[i] = hw_states_[i];
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn AR3SystemPositionOnlyHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type AR3SystemPositionOnlyHardware::read()
{
  // Read the encoder counts
  if (not comm_.get_joint_positions(hw_states_)) {
    RCLCPP_FATAL(
        rclcpp::get_logger("AR3SystemPositionOnlyHardware"),
        "Failed to read joint positions");

  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AR3SystemPositionOnlyHardware::write()
{
  comm_.set_joint_positions(hw_commands_);
  return hardware_interface::return_type::OK;
}

}  // namespace ar3_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ar3_hardware::AR3SystemPositionOnlyHardware, hardware_interface::SystemInterface)
