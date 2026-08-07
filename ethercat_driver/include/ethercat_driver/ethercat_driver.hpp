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
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "transmission_interface/transmission.hpp"
#include "transmission_interface/transmission_loader.hpp"
#include "ethercat_driver/ethercat_bus_manager.hpp"
#include "ethercat_driver/visibility_control.h"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace ethercat_driver
{

class EthercatDriver : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(EthercatDriver)

  ETHERCAT_DRIVER_PUBLIC
  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  ETHERCAT_DRIVER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  ETHERCAT_DRIVER_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ETHERCAT_DRIVER_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ETHERCAT_DRIVER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  ETHERCAT_DRIVER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  ETHERCAT_DRIVER_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;

  ETHERCAT_DRIVER_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

protected:
  std::vector<std::vector<double>> hw_joint_commands_;
  std::vector<std::vector<double>> hw_sensor_commands_;
  std::vector<std::vector<double>> hw_gpio_commands_;
  std::vector<std::vector<double>> hw_joint_states_;
  std::vector<std::vector<double>> hw_sensor_states_;
  std::vector<std::vector<double>> hw_gpio_states_;

  // Sizes the six hw_*_states_/hw_*_commands_ vectors from info_.joints/.sensors/.gpios,
  // NaN-filled, one slot per URDF-declared state/command interface. Factored out of on_init()
  // so it is independently callable (e.g. from a test) without touching bus_manager_.
  void resizeIoBuffers();

  // One entry of a resolved-once (component-interfaces-index -> buffer-index) mapping, used
  // to copy a joint's state interfaces into/out of a transmission role's staging buffer.
  struct TransmissionInterfaceMap
  {
    std::size_t component_index;  // index into ComponentInfo::state_interfaces
    std::size_t buffer_index;     // index into TransmissionRole::buffer
  };

  // Same as TransmissionInterfaceMap, for command interfaces, plus the per-transmission
  // command-interface-name id used to track whether a commanded value failed to propagate.
  struct TransmissionCommandMap
  {
    std::size_t component_index;
    std::size_t buffer_index;
    std::size_t name_id;
  };

  // One <joint> or <actuator> entry of a loaded <transmission>: a staging buffer sized to the
  // union of that joint's declared state_interfaces and command_interfaces names (not a fixed
  // list), plus the index maps used to copy to/from hw_joint_states_/hw_joint_commands_ every
  // cycle. The buffer is sized once, before transmission handles are bound to it, and never
  // resized afterwards.
  struct TransmissionRole
  {
    std::size_t joint_index;     // index into info_.joints / hw_joint_states_ / hw_joint_commands_
    bool has_ec_module;          // true if this joint has its own <ec_module> (drive-backed)
    std::vector<double> buffer;
    std::vector<std::string> interface_names;  // buffer_index -> name
    std::vector<TransmissionInterfaceMap> state_index_map;
    std::vector<TransmissionCommandMap> command_index_map;
  };

  // One loaded <transmission>: the plugin instance, its joint/actuator role buffers, and the
  // once-only warning latch for the command-projection fallback.
  struct TransmissionRuntime
  {
    std::shared_ptr<transmission_interface::Transmission> transmission;
    std::string name;
    std::vector<TransmissionRole> joints;
    std::vector<TransmissionRole> actuators;
    std::size_t num_command_names{0};
    std::vector<bool> commanded_by_name;  // scratch, cleared every write() cycle
    std::vector<bool> dropped_by_name;    // scratch, cleared every write() cycle
    bool warned_command_fallback{false};
  };

  // Builds one TransmissionRole for `joint_name`. Returns false (and logs) on an unresolvable
  // joint name or an actuator role naming a joint with no <ec_module>. A member function (not
  // a free function) because TransmissionRole is a protected nested type.
  bool buildTransmissionRole(
    const std::unordered_map<std::string, std::size_t> & joint_index,
    std::unordered_map<std::string, std::size_t> & command_name_ids,
    const std::string & joint_name, bool is_actuator, TransmissionRole & role_out);

  // Load and configure every <transmission> declared in the URDF <ros2_control> section.
  // Requires info_ to be populated; does not touch bus_manager_.
  bool loadTransmissions();

  // Read direction: hw_joint_states_ of actuator-role joints -> transmission ->
  // hw_joint_states_ of transmission-only joint roles. Call after bus_manager_.read() has
  // populated hw_joint_states_ from the wire.
  void propagateTransmissionStates();

  // Write direction: hw_joint_commands_ of transmission-only joint roles -> transmission ->
  // hw_joint_commands_ of actuator-role joints. Call before bus_manager_.write() sends
  // hw_joint_commands_ over the wire.
  void applyTransmissionCommands();

  std::vector<TransmissionRuntime> transmissions_;
  std::unique_ptr<pluginlib::ClassLoader<transmission_interface::TransmissionLoader>>
  transmission_loader_;

  EthercatBusManager bus_manager_;
};
}  // namespace ethercat_driver

#endif  // ETHERCAT_DRIVER__ETHERCAT_DRIVER_HPP_
