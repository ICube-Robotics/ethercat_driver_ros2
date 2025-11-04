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

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ethercat_driver
{

unsigned int uint_from_string(const std::string & str)
{
  // Strip leading and trailing whitespaces
  std::string s = std::regex_replace(str, std::regex("^ +| +$|( ) +"), "$1");
  // Test if the number is in hexadecimal format
  if (s.find("0x") == 0) {
    return std::stoul(s, nullptr, 16);
  }
  return std::stoul(s);
}

void getTransferMemoryInfo(
  const YAML::Node & element,
  ethercat_interface::EcMemoryEntry & entry,
  const std::string & dir,
  const std::string & transfer_net_name)
{
  if (!element["ec_module"]) {
    std::string msg = "Transfer definition without ec_module entry, net: " +
      transfer_net_name + " direction: " + dir;
    throw std::runtime_error(msg);
  }
  if (!element["index"]) {
    std::string msg = "Transfer definition without index entry, net: " +
      transfer_net_name + " direction: " + dir;
    throw std::runtime_error(msg);
  }
  if (!element["subindex"]) {
    std::string msg = "Transfer definition without subindex entry, net: " +
      transfer_net_name + " direction: " + dir;
    throw std::runtime_error(msg);
  }

  entry.module_name = element["ec_module"].as<std::string>();
  entry.index = uint_from_string(element["index"].as<std::string>());
  entry.subindex = uint_from_string(element["subindex"].as<std::string>());
}

void throwErrorIfModuleParametersNotFound(
  const ethercat_interface::EcTransferEntry & transfer,
  const std::string & module_name,
  const std::string & transfer_net_name,
  const std::string & direction)
{
  std::string msg = "In transfer net: " + transfer_net_name + ", for transfer " +
    transfer.to_simple_string() + ", the module name of the " + direction + "( " + module_name +
    ") among all the recorded modules.";
  RCLCPP_ERROR(
    rclcpp::get_logger(
      "EthercatDriver"), msg.c_str());
  throw std::runtime_error(msg);
}

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

CallbackReturn EthercatDriver::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  const std::lock_guard<std::mutex> lock(ec_mutex_);
  activated_ = false;

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

  // Setup slave modules defined per joints in the URDF
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

  // Setup slave modules defined per GPIOs in the URDF
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

  // Setup slave modules defined per sensors in the URDF
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

  // Check if a transfer configuration is provided
  if (info_.hardware_parameters.find("fsoe_config") != info_.hardware_parameters.end() ||
    info_.hardware_parameters.find("transfer_config") != info_.hardware_parameters.end())
  {
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Transfer configuration detected, ...");

    YAML::Node config;
    // Load the transfer config file
    loadTransferConfigYamlFile(config);

    // Parse transfer modules from the transfer yaml file
    auto transfer_module_params = getEcTransferModuleParam(config);

    // Append the transfer modules parameters to the list of modules parameters
    size_t idx_1st = ec_module_parameters_.size();
    ec_module_parameters_.insert(
      ec_module_parameters_.end(), transfer_module_params.begin(), transfer_module_params.end());
    for (size_t i = 0; i < transfer_module_params.size(); i++) {
      ec_transfer_slaves_.push_back(idx_1st + i);
    }

    // Parse transfer nets from the transfer yaml file
    ec_transfer_nets_ = getEcTransferNets(config);

    // Append the transfer modules to the list of modules and load them
    for (const auto & transfer_module_param : transfer_module_params) {
      try {
        auto ec_module = ec_loader_.createSharedInstance(transfer_module_param.at("plugin"));
        if (!ec_module->setupSlave(
            transfer_module_param, &empty_interface_, &empty_interface_))
        {
          const std::string & module_name = transfer_module_param.at("name");
          RCLCPP_FATAL(
            rclcpp::get_logger("EthercatDriver"),
            "Setup of transfer only module %s FAILED.", module_name.c_str() );
          return CallbackReturn::ERROR;
        }

        auto idx = ec_modules_.size();
        ec_module->setAliasAndPosition(
          getAliasOrDefaultAlias(transfer_module_param),
          std::stoul(transfer_module_param.at("position")));
        ec_modules_.push_back(ec_module);
        ec_transfer_slaves_.push_back(idx);
      } catch (const pluginlib::PluginlibException & ex) {
        const std::string & module_name = transfer_module_param.at("name");
        RCLCPP_ERROR(
          rclcpp::get_logger(
            "EthercatDriver"),
          "The plugin failed to load for transfer module %s. Error: %s\n",
          module_name.c_str(), ex.what());
      }
    }

    // Find all masters from the nets
    {
      std::vector<std::string> master_names;
      for (const auto & net : ec_transfer_nets_) {
        master_names.push_back(net.master);
      }
      for (size_t i = 0; i < ec_module_parameters_.size(); i++) {
        if (std::find(
            master_names.begin(), master_names.end(),
            ec_module_parameters_[i].at("name")) !=
          master_names.end())
        {
          ec_transfer_masters_.push_back(i);
        }
      }
    }

    // Identify (alias,position) all the modules participating in transfers
    for (auto & net : ec_transfer_nets_) {
      for (auto & transfer : net.transfers) {
        // Update each EcMemoryEntry with the alias and position of the module
        size_t in_idx = ec_module_parameters_.size();
        for (in_idx = 0; in_idx < ec_module_parameters_.size(); ++in_idx) {
          if (ec_module_parameters_[in_idx].at("name") == transfer.input.module_name) {
            break;
          }
        }
        size_t out_idx = ec_module_parameters_.size();
        for (out_idx = 0; out_idx < ec_module_parameters_.size(); ++out_idx) {
          if (ec_module_parameters_[out_idx].at("name") == transfer.output.module_name) {
            break;
          }
        }
        if (in_idx == ec_module_parameters_.size()) {
          throwErrorIfModuleParametersNotFound(
            transfer, transfer.input.module_name, net.name, "input");
        }
        if (out_idx == ec_module_parameters_.size()) {
          throwErrorIfModuleParametersNotFound(
            transfer, transfer.output.module_name, net.name, "output");
        }

        const auto & input_module = ec_modules_[in_idx];
        const auto & output_module = ec_modules_[out_idx];

        transfer.input.alias = input_module->alias_;
        transfer.input.position = input_module->position_;
        transfer.output.alias = output_module->alias_;
        transfer.output.position = output_module->position_;
      }
    }

    RCLCPP_INFO(
      rclcpp::get_logger("EthercatDriver"),
      "Transfer configuration loaded successfully!");
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

  // Configure transfer network if transfer nets are defined
  if (!ec_transfer_nets_.empty()) {
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Configuring transfer network...");
    master_->registerTransferInDomain(ec_transfer_nets_);
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Transfer network configured!");
  }

  // start after one second
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  t.tv_sec++;

  bool running = true;
  while (running) {
    // wait until next shot
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
    // update EtherCAT bus

    master_->update();

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
    master_->readData();
  }
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

void EthercatDriver::loadTransferConfigYamlFile(YAML::Node & node, const std::string & path)
{
  std::string file_path;
  if (path.empty()) {
    // Get the fsoe_config or transfer_config parameter of the ethercat_driver hardware plugin
    if (info_.hardware_parameters.find("fsoe_config") == info_.hardware_parameters.end() &&
      info_.hardware_parameters.find("transfer_config") == info_.hardware_parameters.end() )
    {
      std::string msg("transfer_config or fsoe_config parameter is missing!");
      // Transfer (or fsoe) config file was not provided
      RCLCPP_FATAL(
        rclcpp::get_logger("EthercatDriver"), msg.c_str());
      throw std::runtime_error(msg);
    }
    if (info_.hardware_parameters.find("fsoe_config") != info_.hardware_parameters.end() &&
      info_.hardware_parameters.find("transfer_config") != info_.hardware_parameters.end())
    {
      std::string msg(
        "Both transfer_config and fsoe_config parameters are provided! Please provide only one "
        "of them.");
      RCLCPP_FATAL(
        rclcpp::get_logger("EthercatDriver"), msg.c_str());
      throw std::runtime_error(msg);
    }
    if (info_.hardware_parameters.find("fsoe_config") != info_.hardware_parameters.end() ) {
      std::string msg("The fsoe_config parameter is deprecated. "
        "Please use transfer_config instead.");
      RCLCPP_WARN(
        rclcpp::get_logger("EthercatDriver"), msg.c_str());
      file_path = info_.hardware_parameters.at("fsoe_config");
    }
    if (info_.hardware_parameters.find("transfer_config") != info_.hardware_parameters.end()) {
      file_path = info_.hardware_parameters.at("transfer_config");
    }
  } else {
    file_path = path;
  }

  try {
    node = YAML::LoadFile(file_path);
  } catch (const YAML::ParserException & ex) {
    std::string msg =
      std::string(
      "EthercatDriver : failed to load transfer configuration "
      "(YAML file is incorrect): ") + std::string(ex.what());
    RCLCPP_FATAL(
      rclcpp::get_logger("EthercatDriver"), msg.c_str() );
    throw std::runtime_error(msg);
  } catch (const YAML::BadFile & ex) {
    std::string msg =
      std::string(
      "EthercatDriver : failed to load transfer configuration "
      "(file path is incorrect or file is damaged): " + std::string(ex.what()));
    RCLCPP_FATAL(
      rclcpp::get_logger("EthercatDriver"), msg.c_str() );
    throw std::runtime_error(msg);
  } catch (std::exception & e) {
    std::string msg =
      std::string(
      "EthercatDriver : error while loading transfer configuration: ") + std::string(e.what());
    RCLCPP_FATAL(
      rclcpp::get_logger("EthercatDriver"), msg.c_str() );
    throw std::runtime_error(msg);
  }
}

std::vector<std::unordered_map<std::string, std::string>> EthercatDriver::getEcTransferModuleParam(
  const YAML::Node & config)
{
  if (0 == config.size() ) {
    std::string msg = "Empty transfer_config or fsoe_config parameter!";
    RCLCPP_FATAL(
      rclcpp::get_logger("EthercatDriver"), msg.c_str());
    throw std::runtime_error(msg);
  }
  std::vector<std::unordered_map<std::string, std::string>> module_params;
  std::unordered_map<std::string, std::string> module_param;

  // It is possible that modules are only involved in transfers and hence
  // not declared in the ros2_control xacro file.
  // This is a common situation with modules only involved in safety
  // operations. In this case, it is necessary to find the plugin to load,
  // the position and the alias for those slaves.
  if (config["transfer_modules"]) {
    for (const auto & module : config["transfer_modules"]) {
      module_param.clear();
      module_param["name"] = module["name"].as<std::string>();
      module_param["plugin"] = module["plugin"].as<std::string>();
      for (const auto & param : module["parameters"]) {
        module_param[param.first.as<std::string>()] = param.second.as<std::string>();
      }
      module_params.push_back(module_param);
    }
  }

  if (config["safety_modules"]) {
    for (const auto & module : config["safety_modules"]) {
      module_param.clear();
      module_param["name"] = module["name"].as<std::string>();
      module_param["plugin"] = module["plugin"].as<std::string>();
      for (const auto & param : module["parameters"]) {
        module_param[param.first.as<std::string>()] = param.second.as<std::string>();
      }
      module_params.push_back(module_param);
    }
  }

  return module_params;
}

std::vector<ethercat_interface::EcTransferNet> EthercatDriver::getEcTransferNets(
  const YAML::Node & config)
{
  if (0 == config.size() ) {
    std::string msg = "Empty transfer_config or fsoe_config parameter!";
    RCLCPP_FATAL(
      rclcpp::get_logger("EthercatDriver"), msg.c_str());
    throw std::runtime_error(msg);
  }

  std::vector<ethercat_interface::EcTransferNet> transfer_nets;
  ethercat_interface::EcTransferNet transfer_net;

  if (config["nets"]) {
    for (const auto & net : config["nets"]) {
      transfer_net.reset(net["name"].as<std::string>());
      if (net["safety_master"]) {
        transfer_net.master = net["safety_master"].as<std::string>();
      }
      if (net["transfer_master"]) {
        transfer_net.master = net["transfer_master"].as<std::string>();
      }
      for (const auto & transfer : net["transfers"]) {
        ethercat_interface::EcTransferEntry transfer_entry;
        if (!transfer["size"]) {
          std::string msg = "ERROR: transfer n°" + std::to_string(transfer_nets.size()) +
            " of net " +
            transfer_net.name + " : definition without «size» parameter";
          RCLCPP_FATAL(
            rclcpp::get_logger("EthercatDriver"), msg.c_str());
          throw std::runtime_error(msg);
        }
        if (!transfer["in"]) {
          std::string msg = "ERROR: transfer n°" + std::to_string(transfer_nets.size()) +
            " of net " +
            transfer_net.name + " : definition without «in» parameter";
          RCLCPP_FATAL(
            rclcpp::get_logger("EthercatDriver"), msg.c_str());
          throw std::runtime_error(msg);
        }
        if (!transfer["out"]) {
          std::string msg = "ERROR: transfer n°" + std::to_string(transfer_nets.size()) +
            " of net " +
            transfer_net.name + " : definition without «out» parameter";
          RCLCPP_FATAL(
            rclcpp::get_logger("EthercatDriver"), msg.c_str());
          throw std::runtime_error(msg);
        }
        transfer_entry.size = transfer["size"].as<size_t>();
        getTransferMemoryInfo(
          transfer["in"], transfer_entry.input,
          "in", transfer_net.name);
        getTransferMemoryInfo(
          transfer["out"], transfer_entry.output,
          "out", transfer_net.name);
        transfer_net.transfers.push_back(transfer_entry);
      }
      transfer_nets.push_back(transfer_net);
    }
  }

  return transfer_nets;
}

void EthercatDriver::configTransferNetwork()
{
  // This method can be used for additional transfer network configuration if needed
  // Currently, transfer network configuration is handled in on_activate()
}

}  // namespace ethercat_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ethercat_driver::EthercatDriver, hardware_interface::SystemInterface)
