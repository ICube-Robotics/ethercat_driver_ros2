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

#include "ethercat_driver/ethercat_driver.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>

#include "ethercat_driver/ethercat_ros2_control_xml_parser.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "transmission_interface/handle.hpp"

namespace ethercat_driver
{
namespace
{

ConfiguredEcModule make_configured_ec_module(
  std::unordered_map<std::string, std::string> parameters,
  std::vector<double> * input_values,
  std::vector<double> * output_values,
  std::string component_name,
  std::string module_type,
  size_t module_number)
{
  ConfiguredEcModule module;
  module.parameters = std::move(parameters);
  module.input_values = input_values;
  module.output_values = output_values;
  module.component_name = std::move(component_name);
  module.module_type = std::move(module_type);
  module.module_number = module_number;
  return module;
}

bool configure_ethercat_bus_config(
  const std::unordered_map<std::string, std::string> & hardware_parameters,
  EthercatBusConfig & bus_config)
{
  std::string master_iface = "666";

  // Get master id
  if (hardware_parameters.find("master_id") == hardware_parameters.end()) {
    // Master id was not provided, default to 0
    master_iface.assign("0");
  } else {
    try {
      master_iface = hardware_parameters.at("master_id");
    } catch (std::exception & e) {
      RCLCPP_FATAL(
        rclcpp::get_logger("EthercatDriver"), "Invalid master id (%s)!", e.what());
      return false;
    }
  }

  // Get master plugin from hardware description parameter "master_plugin".
  // Default master plugin is EtherlabMaster
  std::string master_plugin_name = "ethercat_master/EtherlabMaster";
  if (hardware_parameters.find("master_plugin") == hardware_parameters.end()) {
    // Master plugin was not provided, default to EtherlabMaster
    master_plugin_name.assign("ethercat_master/EtherlabMaster");
  } else {
    try {
      master_plugin_name = hardware_parameters.at("master_plugin");
    } catch (std::exception & e) {
      RCLCPP_FATAL(
        rclcpp::get_logger("EthercatDriver"), "Invalid master plugin (%s)!", e.what());
      return false;
    }
  }

  bus_config.master_iface = master_iface;
  bus_config.master_plugin = master_plugin_name;


  // Get control frequency
  if (hardware_parameters.find("control_frequency") == hardware_parameters.end()) {
    // Control frequency was not provided, default to 100 Hz
    bus_config.control_frequency = 100.0;
  } else {
    try {
      bus_config.control_frequency = std::stod(hardware_parameters.at("control_frequency"));
    } catch (std::exception & e) {
      RCLCPP_FATAL(
        rclcpp::get_logger("EthercatDriver"), "Invalid control frequency (%s)!", e.what());
      return false;
    }
  }

  // Get readiness timeout (bound on activateBus()'s wait for all slaves to reach
  // OPERATIONAL)
  if (hardware_parameters.find("readiness_timeout_s") == hardware_parameters.end()) {
    bus_config.readiness_timeout_s = 25.0;
  } else {
    try {
      bus_config.readiness_timeout_s = std::stod(hardware_parameters.at("readiness_timeout_s"));
    } catch (std::exception & e) {
      RCLCPP_FATAL(
        rclcpp::get_logger("EthercatDriver"), "Invalid readiness timeout (%s)!", e.what());
      return false;
    }
  }

  const auto transfer_config = hardware_parameters.find("transfer_config");
  const auto fsoe_config = hardware_parameters.find("fsoe_config");
  if (transfer_config != hardware_parameters.end() && fsoe_config != hardware_parameters.end()) {
    RCLCPP_FATAL(
      rclcpp::get_logger("EthercatDriver"),
      "Both transfer_config and fsoe_config parameters are provided! Please provide only one "
      "of them.");
    return false;
  }
  if (transfer_config != hardware_parameters.end() && transfer_config->second.empty()) {
    RCLCPP_FATAL(
      rclcpp::get_logger("EthercatDriver"), "Empty transfer_config or fsoe_config parameter!");
    return false;
  }
  if (fsoe_config != hardware_parameters.end() && fsoe_config->second.empty()) {
    RCLCPP_FATAL(
      rclcpp::get_logger("EthercatDriver"), "Empty transfer_config or fsoe_config parameter!");
    return false;
  }

  if (transfer_config != hardware_parameters.end()) {
    bus_config.transfer_config = transfer_config->second;
  }

  if (fsoe_config != hardware_parameters.end()) {
    bus_config.fsoe_config = fsoe_config->second;
  }

  return true;
}

}  // namespace

void EthercatDriver::resizeIoBuffers()
{
  // Set state vectors
  hw_joint_states_.resize(info_.joints.size());
  for (uint j = 0; j < info_.joints.size(); j++) {
    hw_joint_states_[j].resize(
      info_.joints[j].state_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
  }
  hw_sensor_states_.resize(info_.sensors.size());
  for (uint s = 0; s < info_.sensors.size(); s++) {
    hw_sensor_states_[s].resize(
      info_.sensors[s].state_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
  }
  hw_gpio_states_.resize(info_.gpios.size());
  for (uint g = 0; g < info_.gpios.size(); g++) {
    hw_gpio_states_[g].resize(
      info_.gpios[g].state_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
  }

  // Set command vectors
  hw_joint_commands_.resize(info_.joints.size());
  for (uint j = 0; j < info_.joints.size(); j++) {
    hw_joint_commands_[j].resize(
      info_.joints[j].command_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
  }
  hw_sensor_commands_.resize(info_.sensors.size());
  for (uint s = 0; s < info_.sensors.size(); s++) {
    hw_sensor_commands_[s].resize(
      info_.sensors[s].command_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
  }
  hw_gpio_commands_.resize(info_.gpios.size());
  for (uint g = 0; g < info_.gpios.size(); g++) {
    hw_gpio_commands_[g].resize(
      info_.gpios[g].command_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
  }
}

bool EthercatDriver::buildTransmissionRole(
  const std::unordered_map<std::string, std::size_t> & joint_index,
  std::unordered_map<std::string, std::size_t> & command_name_ids,
  const std::string & joint_name, bool is_actuator, TransmissionRole & role_out)
{
  const auto it = joint_index.find(joint_name);
  if (it == joint_index.end()) {
    RCLCPP_ERROR(rclcpp::get_logger("EthercatDriver"),
      "Transmission references unknown joint '%s'.", joint_name.c_str());
    return false;
  }
  const std::size_t j = it->second;
  const bool has_ec_module = !getEcModuleParam(info_.original_xml, joint_name, "joint").empty();
  if (is_actuator && !has_ec_module) {
    RCLCPP_ERROR(rclcpp::get_logger("EthercatDriver"),
      "Transmission actuator '%s' must name a joint with an <ec_module> (drive-backed).",
      joint_name.c_str());
    return false;
  }

  role_out.joint_index = j;
  role_out.has_ec_module = has_ec_module;

  // The staging buffer is the union of this joint's declared state and command interface
  // names, in first-seen order - not a fixed list. A transmission plugin that needs an
  // interface no other joint uses gets it automatically, as soon as the URDF declares it here.
  std::unordered_map<std::string, std::size_t> name_to_slot;
  const auto & sci = info_.joints[j].state_interfaces;
  const auto & cci = info_.joints[j].command_interfaces;
  role_out.state_index_map.reserve(sci.size());
  role_out.command_index_map.reserve(cci.size());

  for (std::size_t k = 0; k < sci.size(); ++k) {
    const auto [slot_it, inserted] = name_to_slot.try_emplace(sci[k].name, name_to_slot.size());
    (void)inserted;
    role_out.state_index_map.push_back({k, slot_it->second});
  }
  for (std::size_t k = 0; k < cci.size(); ++k) {
    const auto [slot_it, inserted] = name_to_slot.try_emplace(cci[k].name, name_to_slot.size());
    (void)inserted;
    const auto [id_it, id_inserted] = command_name_ids.try_emplace(
      cci[k].name, command_name_ids.size());
    (void)id_inserted;
    role_out.command_index_map.push_back({k, slot_it->second, id_it->second});
  }

  role_out.buffer.assign(name_to_slot.size(), 0.0);
  role_out.interface_names.resize(name_to_slot.size());
  for (const auto & [name, slot] : name_to_slot) {role_out.interface_names[slot] = name;}
  return true;
}

bool EthercatDriver::loadTransmissions()
{
  std::unordered_map<std::string, std::size_t> joint_index;
  joint_index.reserve(info_.joints.size());
  for (std::size_t j = 0; j < info_.joints.size(); ++j) {
    joint_index.emplace(info_.joints[j].name, j);
  }

  transmission_loader_ = std::make_unique<
    pluginlib::ClassLoader<transmission_interface::TransmissionLoader>>(
    "transmission_interface", "transmission_interface::TransmissionLoader");
  transmissions_.clear();
  transmissions_.reserve(info_.transmissions.size());

  for (auto tx_info : info_.transmissions) {  // by value: a parameter is injected below
    // Transmission loaders that extract linkage geometry (e.g. a four-bar-linkage type)
    // parse this from the URDF kinematic tree.
    tx_info.parameters["__robot_description"] = info_.original_xml;

    TransmissionRuntime rt;
    rt.name = tx_info.name;

    try {
      const auto loader = transmission_loader_->createSharedInstance(tx_info.type);
      rt.transmission = loader->load(tx_info);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger("EthercatDriver"),
        "Loading transmission '%s' (type '%s') failed: %s",
        tx_info.name.c_str(), tx_info.type.c_str(), e.what());
      return false;
    }
    if (!rt.transmission) {
      RCLCPP_ERROR(rclcpp::get_logger("EthercatDriver"),
        "Transmission loader for '%s' returned null.", tx_info.name.c_str());
      return false;
    }

    std::unordered_map<std::string, std::size_t> command_name_ids;

    for (const auto & jt : tx_info.joints) {
      TransmissionRole role;
      if (!buildTransmissionRole(joint_index, command_name_ids, jt.name, false, role)) {
        return false;
      }
      rt.joints.push_back(std::move(role));
    }
    for (const auto & act : tx_info.actuators) {
      TransmissionRole role;
      if (!buildTransmissionRole(joint_index, command_name_ids, act.name, true, role)) {
        return false;
      }
      rt.actuators.push_back(std::move(role));
    }
    rt.num_command_names = command_name_ids.size();
    rt.commanded_by_name.assign(rt.num_command_names, false);
    rt.dropped_by_name.assign(rt.num_command_names, false);

    // Handles: one per buffer slot, in declaration order. No interface name is hardcoded -
    // the set offered to the plugin is exactly what buildTransmissionRole() resolved above.
    std::vector<transmission_interface::JointHandle> joint_handles;
    for (auto & role : rt.joints) {
      const std::string & name = info_.joints[role.joint_index].name;
      for (std::size_t s = 0; s < role.buffer.size(); ++s) {
        joint_handles.emplace_back(name, role.interface_names[s], &role.buffer[s]);
      }
    }
    std::vector<transmission_interface::ActuatorHandle> actuator_handles;
    for (auto & role : rt.actuators) {
      const std::string & name = info_.joints[role.joint_index].name;
      for (std::size_t s = 0; s < role.buffer.size(); ++s) {
        actuator_handles.emplace_back(name, role.interface_names[s], &role.buffer[s]);
      }
    }

    try {
      rt.transmission->configure(joint_handles, actuator_handles);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger("EthercatDriver"),
        "Configuring transmission '%s' failed: %s", tx_info.name.c_str(), e.what());
      return false;
    }

    transmissions_.push_back(std::move(rt));
  }
  return true;
}

void EthercatDriver::propagateTransmissionStates()
{
  for (auto & tx : transmissions_) {
    for (auto & role : tx.actuators) {
      const auto & hw = hw_joint_states_[role.joint_index];
      for (const auto & m : role.state_index_map) {
        role.buffer[m.buffer_index] = hw[m.component_index];
      }
    }
    tx.transmission->actuator_to_joint();
    for (auto & role : tx.joints) {
      if (role.has_ec_module) {continue;}  // drive-backed: keep the measured state
      auto & hw = hw_joint_states_[role.joint_index];
      for (const auto & m : role.state_index_map) {
        hw[m.component_index] = role.buffer[m.buffer_index];
      }
    }
  }
}

void EthercatDriver::applyTransmissionCommands()
{
  constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
  for (auto & tx : transmissions_) {
    std::fill(tx.commanded_by_name.begin(), tx.commanded_by_name.end(), false);
    std::fill(tx.dropped_by_name.begin(), tx.dropped_by_name.end(), false);

    // Stage transmission-only joints' commands. A joint that also has its own <ec_module> is
    // commanded directly and is not routed through the transmission.
    for (auto & role : tx.joints) {
      if (role.has_ec_module) {continue;}
      const auto & hw = hw_joint_commands_[role.joint_index];
      for (const auto & m : role.command_index_map) {
        const double v = hw[m.component_index];
        role.buffer[m.buffer_index] = v;
        tx.commanded_by_name[m.name_id] = tx.commanded_by_name[m.name_id] || std::isfinite(v);
      }
    }

    // NaN-seed the actuator roles' command slots: whatever is still NaN after
    // joint_to_actuator() was not projected by the loaded transmission plugin.
    for (auto & role : tx.actuators) {
      for (const auto & m : role.command_index_map) {role.buffer[m.buffer_index] = kNan;}
    }

    tx.transmission->joint_to_actuator();

    // Destage: only overwrite hw_joint_commands_ for finite results. A still-NaN slot holds
    // its previous commanded value instead of being clobbered.
    bool any_dropped = false;
    for (auto & role : tx.actuators) {
      auto & hw = hw_joint_commands_[role.joint_index];
      for (const auto & m : role.command_index_map) {
        const double v = role.buffer[m.buffer_index];
        if (std::isfinite(v)) {
          hw[m.component_index] = v;
        } else {
          tx.dropped_by_name[m.name_id] = true;
        }
      }
    }
    for (std::size_t id = 0; id < tx.num_command_names; ++id) {
      any_dropped = any_dropped || (tx.commanded_by_name[id] && tx.dropped_by_name[id]);
    }
    if (any_dropped && !tx.warned_command_fallback) {
      tx.warned_command_fallback = true;
      RCLCPP_WARN(rclcpp::get_logger("EthercatDriver"),
        "Transmission '%s': at least one commanded joint-space interface was not projected to "
        "actuator space by the loaded transmission plugin. The affected actuator command(s) "
        "hold their previous value. Warned once.", tx.name.c_str());
    }
  }
}

CallbackReturn EthercatDriver::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  resizeIoBuffers();

  std::vector<ConfiguredEcModule> configured_modules;

  // Setup slave modules defined per joints in the URDF
  for (uint j = 0; j < info_.joints.size(); j++) {
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "joints");
    // check all joints for EC modules and load into ec_modules_
    auto module_params = getEcModuleParam(
      info_.original_xml, info_.joints[j].name, "joint");
    for (auto i = 0ul; i < module_params.size(); i++) {
      for (auto k = 0ul; k < info_.joints[j].state_interfaces.size(); k++) {
        module_params[i]["state_interface/" +
          info_.joints[j].state_interfaces[k].name] = std::to_string(k);
      }
      for (auto k = 0ul; k < info_.joints[j].command_interfaces.size(); k++) {
        module_params[i]["command_interface/" +
          info_.joints[j].command_interfaces[k].name] = std::to_string(k);
      }
      configured_modules.push_back(
        make_configured_ec_module(
          module_params[i],
          &hw_joint_states_[j],
          &hw_joint_commands_[j],
          info_.joints[j].name,
          "Joint",
          i + 1));
    }
  }

  // Setup slave modules defined per GPIOs in the URDF
  for (uint g = 0; g < info_.gpios.size(); g++) {
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "gpios");
    // check all gpios for EC modules and load into ec_modules_
    auto module_params = getEcModuleParam(
      info_.original_xml, info_.gpios[g].name, "gpio");
    for (auto i = 0ul; i < module_params.size(); i++) {
      for (auto k = 0ul; k < info_.gpios[g].state_interfaces.size(); k++) {
        module_params[i]["state_interface/" +
          info_.gpios[g].state_interfaces[k].name] = std::to_string(k);
      }
      for (auto k = 0ul; k < info_.gpios[g].command_interfaces.size(); k++) {
        module_params[i]["command_interface/" +
          info_.gpios[g].command_interfaces[k].name] = std::to_string(k);
      }
      configured_modules.push_back(
        make_configured_ec_module(
          module_params[i],
          &hw_gpio_states_[g],
          &hw_gpio_commands_[g],
          info_.gpios[g].name,
          "GPIO",
          i + 1));
    }
  }

  // Setup slave modules defined per sensors in the URDF
  for (uint s = 0; s < info_.sensors.size(); s++) {
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "sensors");
    // check all sensors for EC modules and load into ec_modules_
    auto module_params = getEcModuleParam(
      info_.original_xml, info_.sensors[s].name, "sensor");
    for (auto i = 0ul; i < module_params.size(); i++) {
      for (auto k = 0ul; k < info_.sensors[s].state_interfaces.size(); k++) {
        module_params[i]["state_interface/" +
          info_.sensors[s].state_interfaces[k].name] = std::to_string(k);
      }
      for (auto k = 0ul; k < info_.sensors[s].command_interfaces.size(); k++) {
        module_params[i]["command_interface/" +
          info_.sensors[s].command_interfaces[k].name] = std::to_string(k);
      }
      configured_modules.push_back(
        make_configured_ec_module(
          module_params[i],
          &hw_sensor_states_[s],
          &hw_sensor_commands_[s],
          info_.sensors[s].name,
          "Sensor",
          i + 1));
    }
  }

  EthercatBusConfig bus_config;
  if (!configure_ethercat_bus_config(info_.hardware_parameters, bus_config)) {
    return CallbackReturn::ERROR;
  }

  if (!bus_manager_.configureModules(bus_config, configured_modules)) {
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn EthercatDriver::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
EthercatDriver::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  // export joint state interface
  for (uint j = 0; j < info_.joints.size(); j++) {
    for (uint i = 0; i < info_.joints[j].state_interfaces.size(); i++) {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          info_.joints[j].name,
          info_.joints[j].state_interfaces[i].name,
          &hw_joint_states_[j][i]));
    }
  }
  // export sensor state interface
  for (uint s = 0; s < info_.sensors.size(); s++) {
    for (uint i = 0; i < info_.sensors[s].state_interfaces.size(); i++) {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          info_.sensors[s].name,
          info_.sensors[s].state_interfaces[i].name,
          &hw_sensor_states_[s][i]));
    }
  }
  // export gpio state interface
  for (uint g = 0; g < info_.gpios.size(); g++) {
    for (uint i = 0; i < info_.gpios[g].state_interfaces.size(); i++) {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          info_.gpios[g].name,
          info_.gpios[g].state_interfaces[i].name,
          &hw_gpio_states_[g][i]));
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
EthercatDriver::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  // export joint command interface
  std::vector<double> test;
  for (uint j = 0; j < info_.joints.size(); j++) {
    for (uint i = 0; i < info_.joints[j].command_interfaces.size(); i++) {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          info_.joints[j].name,
          info_.joints[j].command_interfaces[i].name,
          &hw_joint_commands_[j][i]));
    }
  }
  // export sensor command interface
  for (uint s = 0; s < info_.sensors.size(); s++) {
    for (uint i = 0; i < info_.sensors[s].command_interfaces.size(); i++) {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          info_.sensors[s].name,
          info_.sensors[s].command_interfaces[i].name,
          &hw_sensor_commands_[s][i]));
    }
  }
  // export gpio command interface
  for (uint g = 0; g < info_.gpios.size(); g++) {
    for (uint i = 0; i < info_.gpios[g].command_interfaces.size(); i++) {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          info_.gpios[g].name,
          info_.gpios[g].command_interfaces[i].name,
          &hw_gpio_commands_[g][i]));
    }
  }
  return command_interfaces;
}

CallbackReturn EthercatDriver::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!bus_manager_.activateBus()) {
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn EthercatDriver::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  bus_manager_.deactivateBus();

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type EthercatDriver::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  if (bus_manager_.read() == EthercatCycleResult::kError) {
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type EthercatDriver::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  if (bus_manager_.write() == EthercatCycleResult::kError) {
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace ethercat_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ethercat_driver::EthercatDriver, hardware_interface::SystemInterface)
