// Copyright 2022 ICUBE Laboratory, University of Strasbourg
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

#ifndef ETHERCAT_DRIVER__ETHERCAT_DRIVER_HPP_
#define ETHERCAT_DRIVER__ETHERCAT_DRIVER_HPP_

#include <unordered_map>
#include <memory>
#include <string>
#include <vector>
#include <pluginlib/class_loader.hpp>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ethercat_driver/visibility_control.h"
#include "ethercat_interface/ec_slave.hpp"
#include "ethercat_interface/ec_master.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace ethercat_driver
{

class EthercatDriver : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(EthercatDriver)

  ETHERCAT_DRIVER_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  ETHERCAT_DRIVER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  ETHERCAT_DRIVER_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ETHERCAT_DRIVER_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ETHERCAT_DRIVER_PUBLIC
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  ETHERCAT_DRIVER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  ETHERCAT_DRIVER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  ETHERCAT_DRIVER_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;

  ETHERCAT_DRIVER_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

protected:
  std::vector<std::unordered_map<std::string, std::string>> getEcModuleParam(
    const std::string & urdf,
    const std::string & component_name,
    const std::string & component_type);

  uint16_t getAliasOrDefaultAlias(
    const std::unordered_map<std::string,
    std::string> & slave_parameters);

  virtual CallbackReturn setupMaster();

  CallbackReturn configNetwork();

  void loadNumberOfPhysicalDrives();

protected:
  std::vector<std::shared_ptr<ethercat_interface::EcSlave>> ec_modules_;
  std::vector<std::unordered_map<std::string, std::string>> ec_module_parameters_;

  std::vector<std::vector<double>> hw_joint_commands_;
  std::vector<std::vector<double>> hw_sensor_commands_;
  std::vector<std::vector<double>> hw_gpio_commands_;
  std::vector<std::vector<double>> hw_joint_states_;
  std::vector<std::vector<double>> hw_sensor_states_;
  std::vector<std::vector<double>> hw_gpio_states_;

  pluginlib::ClassLoader<ethercat_interface::EcSlave> ec_loader_{
    "ethercat_interface", "ethercat_interface::EcSlave"};

  double control_frequency_;

  std::shared_ptr<ethercat_interface::EcMaster> master_;
  std::mutex ec_mutex_;
  bool activated_;

private:
  // Store the command for the simulated robot (if any)
  int number_of_physical_drives_ = 6;
  int number_of_virtual_drives_ = 0;
  std::vector<double> vt_states_positions;
  std::vector<double> vt_states_efforts;
  std::vector<double> vt_states_velocities;
  std::vector<double> vt_states_mode_of_operation;
  std::vector<double> vt_states_control_word;
  std::vector<double> vt_states_torque_offset;

  std::vector<double> vt_commands_positions;
  std::vector<double> vt_commands_efforts;
  std::vector<double> vt_commands_velocities;
  std::vector<double> vt_commands_mode_of_operation;
  std::vector<double> vt_commands_control_word;
  std::vector<double> vt_commands_torque_offset;

  // Enum defining at which control level we are
  // Dumb way of maintaining the command_interface type per joint.
  enum integration_level_t : std::uint8_t {UNDEFINED = 0, POSITION = 1, VELOCITY = 2, EFFORT = 3};

  // Active control mode for each actuator
  std::vector<integration_level_t> control_level_;

  std::vector<std::chrono::time_point<std::chrono::steady_clock>> timeLastReadJointsValues_;
  double timeLastReadJointsValuesDuration_ = 0.0;
  std::vector<double> lastVelocity_;
};
}  // namespace ethercat_driver

#endif  // ETHERCAT_DRIVER__ETHERCAT_DRIVER_HPP_
