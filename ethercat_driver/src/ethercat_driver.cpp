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

#include <tinyxml2.h>
#include <string>
#include <regex>
#include <rclcpp/logging.hpp>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace virtual_interface
{
/// Constant defining position interface
constexpr char GPIO_IF_MODE_OF_OPERATION[] = "mode_of_operation";
constexpr char GPIO_CONTROL_WORD[] = "control_word";
constexpr char GPIO_TORQUE_OFFSET[] = "torque_offset";
}  // namespace virtual_interface

namespace ethercat_driver
{


uint16_t EthercatDriver::getAliasOrDefaultAlias(
  const std::unordered_map<std::string,
  std::string> & slave_parameters)
{
  if (slave_parameters.find("alias") != slave_parameters.end()) {
    return std::stoul(slave_parameters.at("alias"));
  } else {
    return 0;
  }
}

void EthercatDriver::loadNumberOfPhysicalDrives()
{
  // Get number of physical drives declared in the configuration
  if (info_.hardware_parameters.find("number_of_physical_drives") ==
    info_.hardware_parameters.end())
  {
    // Master id was not provided, default to 6
    number_of_physical_drives_ = 6;
  } else {
    try {
      number_of_physical_drives_ =
        std::stoul(info_.hardware_parameters["number_of_physical_drives"]);
    } catch (std::exception & e) {
      RCLCPP_FATAL(
        rclcpp::get_logger(
          "EthercatDriver"), "Invalid number_of_physical_drives (%s)! using default value",
        e.what());
      number_of_physical_drives_ = 6;
    }
  }
  RCLCPP_INFO(
    rclcpp::get_logger(
      "EthercatDriver"), "number_of_physical_drives loaded: %i", number_of_physical_drives_);

  // Compute the number of virtual drives to simulate
  number_of_virtual_drives_ = info_.joints.size() - number_of_physical_drives_;
  RCLCPP_INFO(
    rclcpp::get_logger(
      "EthercatDriver"), "number_of_virtual_drives loaded: %i", number_of_virtual_drives_);
}

CallbackReturn EthercatDriver::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  const std::lock_guard<std::mutex> lock(ec_mutex_);
  activated_ = false;

  // --------------------- Virtual actuators management (if any) ------------------------
  // Load physical vs virtual drives config
  loadNumberOfPhysicalDrives();

  vt_states_positions.resize(number_of_virtual_drives_, std::numeric_limits<double>::quiet_NaN());
  vt_states_velocities.resize(number_of_virtual_drives_, std::numeric_limits<double>::quiet_NaN());
  vt_states_efforts.resize(number_of_virtual_drives_, std::numeric_limits<double>::quiet_NaN());
  vt_states_mode_of_operation.resize(
    number_of_virtual_drives_,
    std::numeric_limits<double>::quiet_NaN());
  vt_states_control_word.resize(
    number_of_virtual_drives_,
    std::numeric_limits<double>::quiet_NaN());
  vt_states_torque_offset.resize(
    number_of_virtual_drives_,
    std::numeric_limits<double>::quiet_NaN());

  vt_commands_velocities.resize(
    number_of_virtual_drives_,
    std::numeric_limits<double>::quiet_NaN());
  vt_commands_positions.resize(number_of_virtual_drives_, std::numeric_limits<double>::quiet_NaN());
  vt_commands_efforts.resize(number_of_virtual_drives_, std::numeric_limits<double>::quiet_NaN());
  vt_commands_mode_of_operation.resize(
    number_of_virtual_drives_,
    std::numeric_limits<double>::quiet_NaN());
  vt_commands_control_word.resize(
    number_of_virtual_drives_,
    std::numeric_limits<double>::quiet_NaN());
  vt_commands_torque_offset.resize(
    number_of_virtual_drives_,
    std::numeric_limits<double>::quiet_NaN());

  control_level_.resize(number_of_virtual_drives_, integration_level_t::UNDEFINED);

  timeLastReadJointsValues_.resize(number_of_virtual_drives_);
  // init last read joint time
  for (std::size_t i = 0; i < vt_states_positions.size(); i++) {
    timeLastReadJointsValues_[i] = std::chrono::steady_clock::now();
  }

  lastVelocity_.resize(number_of_virtual_drives_);
  // init last joint velocity
  for (std::size_t i = 0; i < vt_states_positions.size(); i++) {
    lastVelocity_[i] = 0.0;
  }
  // -------------------End of virtual actuators management (if any)--------------------

  hw_joint_states_.resize(number_of_physical_drives_);
  for (uint j = 0; j < hw_joint_states_.size(); j++) {
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
  hw_joint_commands_.resize(number_of_physical_drives_);
  for (uint j = 0; j < hw_joint_commands_.size(); j++) {
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

  for (uint j = 0; j < info_.joints.size(); j++) {
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "joints");
    // check all joints for EC modules and load into ec_modules_
    auto module_params = getEcModuleParam(info_.original_xml, info_.joints[j].name, "joint");
    ec_module_parameters_.insert(
      ec_module_parameters_.end(), module_params.begin(), module_params.end());
    for (auto i = 0ul; i < module_params.size(); i++) {
      for (auto k = 0ul; k < info_.joints[j].state_interfaces.size(); k++) {
        module_params[i]["state_interface/" +
          info_.joints[j].state_interfaces[k].name] = std::to_string(k);
      }
      for (auto k = 0ul; k < info_.joints[j].command_interfaces.size(); k++) {
        module_params[i]["command_interface/" +
          info_.joints[j].command_interfaces[k].name] = std::to_string(k);
      }
      try {
        auto module = ec_loader_.createSharedInstance(module_params[i].at("plugin"));
        if (!module->setupSlave(
            module_params[i], &hw_joint_states_[j], &hw_joint_commands_[j]))
        {
          RCLCPP_FATAL(
            rclcpp::get_logger("EthercatDriver"),
            "Setup of Joint module %li FAILED.", i + 1);
          return CallbackReturn::ERROR;
        }
        module->setAliasAndPosition(
          getAliasOrDefaultAlias(module_params[i]),
          std::stoul(module_params[i].at("position")));
        ec_modules_.push_back(module);
      } catch (pluginlib::PluginlibException & ex) {
        RCLCPP_FATAL(
          rclcpp::get_logger("EthercatDriver"),
          "The plugin of %s failed to load for some reason. Error: %s\n",
          info_.joints[j].name.c_str(), ex.what());
      }
    }
  }
  for (uint g = 0; g < info_.gpios.size(); g++) {
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "gpios");
    // check all gpios for EC modules and load into ec_modules_
    auto module_params = getEcModuleParam(info_.original_xml, info_.gpios[g].name, "gpio");
    ec_module_parameters_.insert(
      ec_module_parameters_.end(), module_params.begin(), module_params.end());
    for (auto i = 0ul; i < module_params.size(); i++) {
      for (auto k = 0ul; k < info_.gpios[g].state_interfaces.size(); k++) {
        module_params[i]["state_interface/" +
          info_.gpios[g].state_interfaces[k].name] = std::to_string(k);
      }
      for (auto k = 0ul; k < info_.gpios[g].command_interfaces.size(); k++) {
        module_params[i]["command_interface/" +
          info_.gpios[g].command_interfaces[k].name] = std::to_string(k);
      }
      try {
        auto module = ec_loader_.createSharedInstance(module_params[i].at("plugin"));
        if (!module->setupSlave(
            module_params[i], &hw_gpio_states_[g], &hw_gpio_commands_[g]))
        {
          RCLCPP_FATAL(
            rclcpp::get_logger("EthercatDriver"),
            "Setup of GPIO module %li FAILED.", i + 1);
          return CallbackReturn::ERROR;
        }
        module->setAliasAndPosition(
          getAliasOrDefaultAlias(module_params[i]),
          std::stoul(module_params[i].at("position")));
        ec_modules_.push_back(module);
      } catch (pluginlib::PluginlibException & ex) {
        RCLCPP_FATAL(
          rclcpp::get_logger("EthercatDriver"),
          "The plugin of %s failed to load for some reason. Error: %s\n",
          info_.gpios[g].name.c_str(), ex.what());
      }
    }
  }
  for (uint s = 0; s < info_.sensors.size(); s++) {
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "sensors");
    // check all sensors for EC modules and load into ec_modules_
    auto module_params = getEcModuleParam(info_.original_xml, info_.sensors[s].name, "sensor");
    ec_module_parameters_.insert(
      ec_module_parameters_.end(), module_params.begin(), module_params.end());
    for (auto i = 0ul; i < module_params.size(); i++) {
      for (auto k = 0ul; k < info_.sensors[s].state_interfaces.size(); k++) {
        module_params[i]["state_interface/" +
          info_.sensors[s].state_interfaces[k].name] = std::to_string(k);
      }
      for (auto k = 0ul; k < info_.sensors[s].command_interfaces.size(); k++) {
        module_params[i]["command_interface/" +
          info_.sensors[s].command_interfaces[k].name] = std::to_string(k);
      }
      try {
        auto module = ec_loader_.createSharedInstance(module_params[i].at("plugin"));
        if (!module->setupSlave(
            module_params[i], &hw_sensor_states_[s], &hw_sensor_commands_[s]))
        {
          RCLCPP_FATAL(
            rclcpp::get_logger("EthercatDriver"),
            "Setup of Sensor module %li FAILED.", i + 1);
          return CallbackReturn::ERROR;
        }
        module->setAliasAndPosition(
          getAliasOrDefaultAlias(module_params[i]),
          std::stoul(module_params[i].at("position")));
        ec_modules_.push_back(module);
      } catch (pluginlib::PluginlibException & ex) {
        RCLCPP_FATAL(
          rclcpp::get_logger("EthercatDriver"),
          "The plugin of %s failed to load for some reason. Error: %s\n",
          info_.sensors[s].name.c_str(), ex.what());
      }
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Got %li modules", ec_modules_.size());

  return CallbackReturn::SUCCESS;
}

CallbackReturn EthercatDriver::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (std::size_t i = 0; i < vt_states_positions.size(); i++) {
    vt_states_positions[i] = 0.0;
    vt_states_efforts[i] = 0.0;
    vt_states_velocities[i] = 0.0;
    vt_states_mode_of_operation[i] = 9.0;
    vt_states_control_word[i] = 0.0;
    vt_states_torque_offset[i] = 0.0;
    vt_commands_positions[i] = 0.0;
    vt_commands_velocities[i] = 0.0;
    vt_commands_efforts[i] = 0.0;
    vt_commands_mode_of_operation[i] = 9.0;
    vt_commands_control_word[i] = 0.0;
    vt_commands_torque_offset[i] = 0.0;
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
EthercatDriver::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  // export joint state interface
  for (uint j = 0; j < hw_joint_states_.size(); j++) {
    for (uint i = 0; i < info_.joints[j].state_interfaces.size(); i++) {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          info_.joints[j].name,
          info_.joints[j].state_interfaces[i].name,
          &hw_joint_states_[j][i]));
    }
  }

  // --------------------- Virtual actuators management (if any) ------------------------
  auto hwIfPosition = hardware_interface::HW_IF_POSITION;
  auto hwIfVelocity = hardware_interface::HW_IF_VELOCITY;
  auto hwIfEffort = hardware_interface::HW_IF_EFFORT;
  auto gpioIfModeOfOperation = virtual_interface::GPIO_IF_MODE_OF_OPERATION;
  auto gpioIfControlWord = virtual_interface::GPIO_CONTROL_WORD;
  auto gpioIfTorqueOffset = virtual_interface::GPIO_TORQUE_OFFSET;
  for (std::size_t i = 0; i < vt_states_positions.size(); i++) {
    state_interfaces.emplace_back(
      info_.joints[i + number_of_physical_drives_].name, hwIfPosition,
      &vt_states_positions[i]);
    state_interfaces.emplace_back(
      info_.joints[i + number_of_physical_drives_].name, hwIfVelocity,
      &vt_states_velocities[i]);
    state_interfaces.emplace_back(
      info_.joints[i + number_of_physical_drives_].name, hwIfEffort,
      &vt_states_efforts[i]);
    state_interfaces.emplace_back(
      info_.joints[i + number_of_physical_drives_].name, gpioIfModeOfOperation,
      &vt_states_mode_of_operation[i]);
    state_interfaces.emplace_back(
      info_.joints[i + number_of_physical_drives_].name, gpioIfControlWord,
      &vt_states_control_word[i]);
    state_interfaces.emplace_back(
      info_.joints[i + number_of_physical_drives_].name, gpioIfTorqueOffset,
      &vt_states_torque_offset[i]);
  }
  // ------------------ End of Virtual actuators management (if any) --------------------

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
  for (uint j = 0; j < hw_joint_commands_.size(); j++) {
    for (uint i = 0; i < info_.joints[j].command_interfaces.size(); i++) {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          info_.joints[j].name,
          info_.joints[j].command_interfaces[i].name,
          &hw_joint_commands_[j][i]));
    }
  }

  // --------------------- Virtual actuators management (if any) ------------------------
  auto hwIfPosition = hardware_interface::HW_IF_POSITION;
  auto hwIfVelocity = hardware_interface::HW_IF_VELOCITY;
  auto hwIfEffort = hardware_interface::HW_IF_EFFORT;
  auto gpioIfModeOfOperation = virtual_interface::GPIO_IF_MODE_OF_OPERATION;
  auto gpioIfControlWord = virtual_interface::GPIO_CONTROL_WORD;
  auto gpioIfTorqueOffset = virtual_interface::GPIO_TORQUE_OFFSET;

  for (std::size_t i = 0; i < vt_states_positions.size(); i++) {
    command_interfaces.emplace_back(
      info_.joints[i + number_of_physical_drives_].name, hwIfPosition,
      &vt_commands_positions[i]);
    command_interfaces.emplace_back(
      info_.joints[i + number_of_physical_drives_].name, hwIfVelocity,
      &vt_commands_velocities[i]);
    command_interfaces.emplace_back(
      info_.joints[i + number_of_physical_drives_].name, hwIfEffort,
      &vt_commands_efforts[i]);
    command_interfaces.emplace_back(
      info_.joints[i + number_of_physical_drives_].name, gpioIfModeOfOperation,
      &vt_commands_mode_of_operation[i]);
    command_interfaces.emplace_back(
      info_.joints[i + number_of_physical_drives_].name, gpioIfControlWord,
      &vt_commands_control_word[i]);
    command_interfaces.emplace_back(
      info_.joints[i + number_of_physical_drives_].name, gpioIfTorqueOffset,
      &vt_commands_torque_offset[i]);
  }
  // --------------------- End of Virtual actuators management (if any) ----------------

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

// Only for virtual actuators (if any)
hardware_interface::return_type
EthercatDriver::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  // START_INTERFACES management
  // Prepare for new command modes
  std::vector<integration_level_t> new_modes = {};
  bool virtual_managed_start_interfaces = false;
  for (std::string key : start_interfaces) {
    for (std::size_t i = 0; i < vt_states_positions.size(); i++) {
      if (key ==
        info_.joints[i + number_of_physical_drives_].name + "/" +
        hardware_interface::HW_IF_POSITION)
      {
        new_modes.push_back(integration_level_t::POSITION);
        virtual_managed_start_interfaces = true;
      }
      if (key ==
        info_.joints[i + number_of_physical_drives_].name + "/" +
        hardware_interface::HW_IF_VELOCITY)
      {
        new_modes.push_back(integration_level_t::VELOCITY);
        virtual_managed_start_interfaces = true;
      }
      if (key ==
        info_.joints[i + number_of_physical_drives_].name + "/" +
        hardware_interface::HW_IF_EFFORT)
      {
        new_modes.push_back(integration_level_t::EFFORT);
        virtual_managed_start_interfaces = true;
      }
    }
  }
  // All joints must be given new command mode at the same time
  if (virtual_managed_start_interfaces && new_modes.size() != vt_states_positions.size()) {
    return hardware_interface::return_type::ERROR;
  }
  // Example criteria: All joints must have the same command mode
  if (virtual_managed_start_interfaces &&
    !std::all_of(
      new_modes.begin() + 1, new_modes.end(),
      [&](integration_level_t mode) {return mode == new_modes[0];}))
  {
    return hardware_interface::return_type::ERROR;
  }

  // Set the new command modes
  if (virtual_managed_start_interfaces) {
    for (std::size_t i = 0; i < vt_states_positions.size(); i++) {
      if (control_level_[i] != integration_level_t::UNDEFINED) {
        // Something else is using the joint! Abort!
        return hardware_interface::return_type::ERROR;
      }
      control_level_[i] = new_modes[i];
    }
  }

  // STOP_INTERFACES management
  // Stop motion on all relevant joints that are stopping
  for (std::string key : stop_interfaces) {
    for (std::size_t i = 0; i < vt_states_positions.size(); i++) {
      if (key.find(info_.joints[i + number_of_physical_drives_].name) != std::string::npos) {
        vt_commands_velocities[i] = 0;
        vt_commands_efforts[i] = 0;
        control_level_[i] = integration_level_t::UNDEFINED;  // Revert to undefined
      }
    }
  }

  return hardware_interface::return_type::OK;
}

CallbackReturn EthercatDriver::setupMaster()
{
  unsigned int master_id = 666;
  // Get master id
  if (info_.hardware_parameters.find("master_id") == info_.hardware_parameters.end()) {
    // Master id was not provided, default to 0
    master_id = 0;
  } else {
    try {
      master_id = std::stoul(info_.hardware_parameters["master_id"]);
    } catch (std::exception & e) {
      RCLCPP_FATAL(
        rclcpp::get_logger("EthercatDriver"), "Invalid master id (%s)!", e.what());
      return CallbackReturn::ERROR;
    }
  }
  master_ = std::make_shared<ethercat_interface::EcMaster>(master_id);

  return CallbackReturn::SUCCESS;
}

CallbackReturn EthercatDriver::configNetwork()
{
  // Get control frequency
  if (info_.hardware_parameters.find("control_frequency") == info_.hardware_parameters.end()) {
    // Control frequency was not provided, default to 100 Hz
    control_frequency_ = 100.0;
  } else {
    try {
      control_frequency_ = std::stod(info_.hardware_parameters["control_frequency"]);
    } catch (std::exception & e) {
      RCLCPP_FATAL(
        rclcpp::get_logger("EthercatDriver"), "Invalid control frequency (%s)!", e.what());
      return CallbackReturn::ERROR;
    }
  }

  // start EC and wait until state operative

  master_->setCtrlFrequency(control_frequency_);

  for (auto i = 0ul; i < ec_modules_.size(); i++) {
    master_->addSlave(ec_modules_[i].get());
  }

  // configure SDO
  for (auto i = 0ul; i < ec_modules_.size(); i++) {
    for (auto & sdo : ec_modules_[i]->sdo_config) {
      uint32_t abort_code;
      int ret = master_->configSlaveSdo(
        std::stod(ec_module_parameters_[i]["position"]),
        sdo,
        &abort_code);
      if (ret) {
        RCLCPP_INFO(
          rclcpp::get_logger("EthercatDriver"),
          "Failed to download config SDO for module at position %s with Error: %d",
          ec_module_parameters_[i]["position"].c_str(),
          abort_code);
      }
    }
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn EthercatDriver::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const std::lock_guard<std::mutex> lock(ec_mutex_);
  if (activated_) {
    RCLCPP_FATAL(rclcpp::get_logger("EthercatDriver"), "Double on_activate()");
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Starting ...please wait...");

  // setup master
  setupMaster();
  // configure network
  configNetwork();

  if (!master_->activate()) {
    RCLCPP_ERROR(rclcpp::get_logger("EthercatDriver"), "Activate EcMaster failed");
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Activated EcMaster!");

  // --------------------- Virtual actuators management (if any) ------------------------
  for (std::size_t i = 0; i < vt_states_positions.size(); i++) {
    // set some default values for joints
    if (std::isnan(vt_states_positions[i])) {
      vt_states_positions[i] = 0;
    }
    if (std::isnan(vt_states_velocities[i])) {
      vt_states_velocities[i] = 0;
    }
    if (std::isnan(vt_states_efforts[i])) {
      vt_states_efforts[i] = 0;
    }
    if (std::isnan(vt_states_mode_of_operation[i])) {
      vt_states_mode_of_operation[i] = 9;
    }
    if (std::isnan(vt_states_control_word[i])) {
      vt_states_control_word[i] = 0;
    }
    if (std::isnan(vt_states_torque_offset[i])) {
      vt_states_torque_offset[i] = 0;
    }
    if (std::isnan(vt_commands_positions[i])) {
      vt_commands_positions[i] = 0;
    }
    if (std::isnan(vt_commands_velocities[i])) {
      vt_commands_velocities[i] = 0;
    }
    if (std::isnan(vt_commands_efforts[i])) {
      vt_commands_efforts[i] = 0;
    }
    if (std::isnan(vt_commands_mode_of_operation[i])) {
      vt_commands_mode_of_operation[i] = 9;
    }
    if (std::isnan(vt_commands_control_word[i])) {
      vt_commands_control_word[i] = 0;
    }
    if (std::isnan(vt_commands_torque_offset[i])) {
      vt_commands_torque_offset[i] = 0;
    }
  }
  // --------------------- Only for Virtual actuators management (if any) ----------------

  // start after one second
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  t.tv_sec++;

  bool running = true;
  while (running) {
    // wait until next shot
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
    // update EtherCAT bus

    master_->update(rclcpp::get_logger("EthercatDriver"));
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "updated!");

    // check if operational
    bool isAllInit = true;
    for (auto & module : ec_modules_) {
      isAllInit = isAllInit && module->initialized();
    }
    if (isAllInit) {
      running = false;
    }
    // calculate next shot. carry over nanoseconds into microseconds.
    t.tv_nsec += master_->getInterval();
    while (t.tv_nsec >= 1000000000) {
      t.tv_nsec -= 1000000000;
      t.tv_sec++;
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("EthercatDriver"), "System Successfully started!");

  activated_ = true;

  return CallbackReturn::SUCCESS;
}

CallbackReturn EthercatDriver::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const std::lock_guard<std::mutex> lock(ec_mutex_);
  activated_ = false;

  RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Stopping ...please wait...");

  // stop EC and disconnect
  master_->stop();

  RCLCPP_INFO(
    rclcpp::get_logger("EthercatDriver"), "System successfully stopped!");

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type EthercatDriver::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // try to lock so we can avoid blocking the read/write loop on the lock.
  const std::unique_lock<std::mutex> lock(ec_mutex_, std::try_to_lock);
  if (lock.owns_lock() && activated_) {
    master_->readData(rclcpp::get_logger("EthercatDriver"));
  }

  // --------------------- Simulating Virtual actuators behaviour (if any) ----------------------
  if (number_of_virtual_drives_ != 0) {
    for (std::size_t i = 0; i < vt_states_positions.size(); i++) {
      switch (control_level_[i]) {
        case integration_level_t::UNDEFINED:
          RCLCPP_DEBUG(
            rclcpp::get_logger(
              "MultiInterfaceHardware"), "Nothing is using the hardware interface!");
          return hardware_interface::return_type::OK;
        case integration_level_t::POSITION:
          vt_states_velocities[i] = 0;
          if (!std::isnan(vt_commands_positions[i]) && !std::isnan(vt_states_positions[i])) {
            vt_states_positions[i] += (vt_commands_positions[i] - vt_states_positions[i]) / 2.0;
          }
          break;

        case integration_level_t::VELOCITY:
          timeLastReadJointsValuesDuration_ =
            std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - timeLastReadJointsValues_[i])
            .count();
          timeLastReadJointsValues_[i] = std::chrono::steady_clock::now();

          if (!std::isnan(vt_commands_velocities[i])) {
            vt_states_positions[i] += vt_commands_velocities[i] *
              (timeLastReadJointsValuesDuration_ / 1000.0);
            vt_commands_positions[i] = vt_states_positions[i];
            if (vt_commands_velocities[i] != 0.0) {
              RCLCPP_DEBUG(
                rclcpp::get_logger(
                  "MultiInterfaceHardware"),
                "deltaT = %f (ms),  speed = %f(rad/s), pos = %f(deg)",
                timeLastReadJointsValuesDuration_, vt_commands_velocities[i],
                (180.0 * vt_states_positions[i] / M_PI));
            }
          }
          break;

        case integration_level_t::EFFORT:
          vt_states_efforts[i] = 0;
          if (!std::isnan(vt_commands_efforts[i]) && !std::isnan(vt_states_efforts[i])) {
            vt_states_efforts[i] += (vt_commands_efforts[i] - vt_states_efforts[i]) / 2.0;
          }
          break;

        default:
          RCLCPP_INFO(
            rclcpp::get_logger(
              "MultiInterfaceHardware"), "Nothing is using the hardware interface!");
          return hardware_interface::return_type::OK;
      }
    }
  }
  // --------------------- End of Simulating Virtual actuators behaviour (if any) ----------------

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type EthercatDriver::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // try to lock so we can avoid blocking the read/write loop on the lock.
  const std::unique_lock<std::mutex> lock(ec_mutex_, std::try_to_lock);
  if (lock.owns_lock() && activated_) {
    master_->writeData();
  }

  // --------------------- Simulating Virtual actuators behaviour (if any) ----------------------
  if (number_of_virtual_drives_ != 0) {
    for (std::size_t i = 0; i < vt_states_positions.size(); i++) {
      if (vt_commands_velocities[i] != lastVelocity_[i]) {
        RCLCPP_DEBUG(
          rclcpp::get_logger(
            "MultiInterfaceHardware"),
          "Command velocity has changed: new value = %.5f for joint '%s'!",
          vt_commands_velocities[i], info_.joints[i + number_of_physical_drives_].name.c_str());
        lastVelocity_[i] = vt_commands_velocities[i];
      }
    }
  }
  // --------------------- ENd of Simulating Virtual actuators behaviour (if any) ----------------

  return hardware_interface::return_type::OK;
}

std::vector<std::unordered_map<std::string, std::string>> EthercatDriver::getEcModuleParam(
  const std::string & urdf,
  const std::string & component_name,
  const std::string & component_type)
{
  // Check if everything OK with URDF string
  if (urdf.empty()) {
    throw std::runtime_error("empty URDF passed to robot");
  }
  tinyxml2::XMLDocument doc;
  if (!doc.Parse(urdf.c_str()) && doc.Error()) {
    throw std::runtime_error("invalid URDF passed in to robot parser");
  }
  if (doc.Error()) {
    throw std::runtime_error("invalid URDF passed in to robot parser");
  }

  tinyxml2::XMLElement * robot_it = doc.RootElement();
  if (std::string("robot").compare(robot_it->Name())) {
    throw std::runtime_error("the robot tag is not root element in URDF");
  }

  const tinyxml2::XMLElement * ros2_control_it = robot_it->FirstChildElement("ros2_control");
  if (!ros2_control_it) {
    throw std::runtime_error("no ros2_control tag");
  }

  std::vector<std::unordered_map<std::string, std::string>> module_params;
  std::unordered_map<std::string, std::string> module_param;

  while (ros2_control_it) {
    const auto * ros2_control_child_it = ros2_control_it->FirstChildElement(component_type.c_str());
    while (ros2_control_child_it) {
      if (!component_name.compare(ros2_control_child_it->Attribute("name"))) {
        const auto * ec_module_it = ros2_control_child_it->FirstChildElement("ec_module");
        while (ec_module_it) {
          module_param.clear();
          module_param["name"] = ec_module_it->Attribute("name");
          const auto * plugin_it = ec_module_it->FirstChildElement("plugin");
          if (NULL != plugin_it) {
            module_param["plugin"] = plugin_it->GetText();
          }
          const auto * param_it = ec_module_it->FirstChildElement("param");
          while (param_it) {
            module_param[param_it->Attribute("name")] = param_it->GetText();
            param_it = param_it->NextSiblingElement("param");
          }
          module_params.push_back(module_param);
          ec_module_it = ec_module_it->NextSiblingElement("ec_module");
        }
      }
      ros2_control_child_it = ros2_control_child_it->NextSiblingElement(component_type.c_str());
    }
    ros2_control_it = ros2_control_it->NextSiblingElement("ros2_control");
  }

  return module_params;
}

}  // namespace ethercat_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ethercat_driver::EthercatDriver, hardware_interface::SystemInterface)
