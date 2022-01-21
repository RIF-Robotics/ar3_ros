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

#ifndef AR3_HARDWARE__AR3_SYSTEM_POSITION_ONLY_HPP_
#define AR3_HARDWARE__AR3_SYSTEM_POSITION_ONLY_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ar3_hardware/visibility_control.h"
#include "ar3_hardware_driver/ar3_encoder_switch_motor_serial_comm.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace ar3_hardware
{
class AR3SystemPositionOnlyHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(AR3SystemPositionOnlyHardware)

  AR3_HARDWARE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  AR3_HARDWARE_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  AR3_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  AR3_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  AR3_HARDWARE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  AR3_HARDWARE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  AR3_HARDWARE_PUBLIC
  hardware_interface::return_type read() override;

  AR3_HARDWARE_PUBLIC
  hardware_interface::return_type write() override;

private:
  //// Parameters for the AR3 simulation
  //double hw_start_sec_;
  //double hw_stop_sec_;
  //double hw_slowdown_;

  //// Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;

  std::vector<int> joint_encoder_counts_;

  const int pulses_per_revolution_ = 512;
  const double counts_per_revolution_ = pulses_per_revolution_ * 4;

  // Reference: https://www.omc-stepperonline.com/ar3-open-source-robot-package-kit-stepper-motor-driver-power-supply-and-bracket.html
  std::vector<double> gear_ratios_ = {
    10.0,
    50.0,
    50.0,
    13.0 + 212.0 / 289.0,
    1.0, // TODO
    19.0 + 38.0/187.0
  };

  std::vector<int> encoder_directions_ = {
    +1,
    -1,
    +1,
    +1,
    +1,
    -1
  };


  std::vector<int> encoder_zero_positions_ = {
    static_cast<int>(7600 * 5.12),
    static_cast<int>(2322 * 5.12),
    static_cast<int>(0 * 5.12),
    static_cast<int>(7600 * 5.12),
    static_cast<int>(2287 * 2.56),
    static_cast<int>(3312 * 5.12)
  };

  std::string serial_device_;
  int serial_baudrate_;
  std::string firmware_version_;

  ar3_hardware_driver::AR3EncoderSwitchMotorSerialComm comm_;

};

}  // namespace ar3_hardware

#endif  // AR3_HARDWARE__AR3_SYSTEM_POSITION_ONLY_HPP_
