// Copyright 2024 ICUBE Laboratory, University of Strasbourg
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
//
// Author: Manuel YGUEL (yguel.robotics@gmail.com)

#include <string>
#include <regex>

#include "ethercat_driver/ethercat_safety_driver.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#ifndef CLASSM
#define CLASSM EthercatSafetyDriver
#else
#error alias CLASSM already defined!
#endif  //< CLASSM

namespace ethercat_driver
{

void CLASSM::loadFsoeConfigYamlFile(YAML::Node & node, const std::string & path)
{
  std::string file_path;
  if (path.empty()) {
    // Get the fsoe_config parameter of the ethercat_driver hardware plugin
    if (info_.hardware_parameters.find("fsoe_config") == info_.hardware_parameters.end()) {
      std::string msg("fsoe_config parameter is missing!");
      // Safety fsoe config file was not provided
      RCLCPP_FATAL(
        rclcpp::get_logger("EthercatSafetyDriver"), msg.c_str());
      throw std::runtime_error(msg);
    }
    file_path = info_.hardware_parameters.at("fsoe_config");
  } else {
    file_path = path;
  }

  try {
    node = YAML::LoadFile(file_path);
  } catch (const YAML::ParserException & ex) {
    std::string msg =
      std::string(
      "EthercatSafetyDriver : failed to load fsoe configuration "
      "(YAML file is incorrect): ") + std::string(ex.what());
    RCLCPP_FATAL(
      rclcpp::get_logger("EthercatSafetyDriver"), msg.c_str() );
    throw std::runtime_error(msg);
  } catch (const YAML::BadFile & ex) {
    std::string msg =
      std::string(
      "EthercatSafetyDriver : failed to load fsoe configuration "
      "(file path is incorrect or file is damaged): " + std::string(ex.what()));
    RCLCPP_FATAL(
      rclcpp::get_logger("EthercatSafetyDriver"), msg.c_str() );
    throw std::runtime_error(msg);
  } catch (std::exception & e) {
    std::string msg =
      std::string(
      "EthercatSafetyDriver : error while loading fsoe configuration: ") + std::string(e.what());
    RCLCPP_FATAL(
      rclcpp::get_logger("EthercatSafetyDriver"), msg.c_str() );
    throw std::runtime_error(msg);
  }
}

std::vector<std::unordered_map<std::string, std::string>> CLASSM::getEcSafetyModuleParam(
  const YAML::Node & config)
{
  if (0 == config.size() ) {
    std::string msg = "Empty fsoe_config file!";
    RCLCPP_FATAL(
      rclcpp::get_logger("EthercatSafetyDriver"), msg.c_str());
    throw std::runtime_error(msg);
  }
  std::vector<std::unordered_map<std::string, std::string>> module_params;
  std::unordered_map<std::string, std::string> module_param;

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
  const std::string & safety_net_name)
{
  if (!element["ec_module"]) {
    std::string msg = "Transfer definition without ec_module entry, net: " +
      safety_net_name + " direction: " + dir;
    throw std::runtime_error(msg);
  }
  if (!element["index"]) {
    std::string msg = "Transfer definition without index entry, net: " +
      safety_net_name + " direction: " + dir;
    throw std::runtime_error(msg);
  }
  if (!element["subindex"]) {
    std::string msg = "Transfer definition without subindex entry, net: " +
      safety_net_name + " direction: " + dir;
    throw std::runtime_error(msg);
  }

  entry.module_name = element["ec_module"].as<std::string>();
  entry.index = uint_from_string(element["index"].as<std::string>());
  entry.subindex = uint_from_string(element["subindex"].as<std::string>());
}

std::vector<ethercat_interface::EcSafetyNet> CLASSM::getEcSafetyNets(const YAML::Node & config)
{
  if (0 == config.size() ) {
    std::string msg = "Empty fsoe_config file!";
    RCLCPP_FATAL(
      rclcpp::get_logger("EthercatSafetyDriver"), msg.c_str());
    throw std::runtime_error(msg);
  }

  std::vector<ethercat_interface::EcSafetyNet> safety_nets;
  ethercat_interface::EcSafetyNet safety_net;

  if (config["nets"]) {
    for (const auto & net : config["nets"]) {
      safety_net.reset(net["name"].as<std::string>());
      safety_net.master = net["safety_master"].as<std::string>();
      for (const auto & transfer : net["transfers"]) {
        ethercat_interface::EcTransferEntry transfer_entry;
        if (!transfer["size"]) {
          std::string msg = "ERROR: transfer n°" + std::to_string(safety_nets.size()) + " of net " +
            safety_net.name + " : definition without «size» parameter";
          RCLCPP_FATAL(
            rclcpp::get_logger("EthercatSafetyDriver"), msg.c_str());
          throw std::runtime_error(msg);
        }
        if (!transfer["in"]) {
          std::string msg = "ERROR: transfer n°" + std::to_string(safety_nets.size()) + " of net " +
            safety_net.name + " : definition without «in» parameter";
          RCLCPP_FATAL(
            rclcpp::get_logger("EthercatSafetyDriver"), msg.c_str());
          throw std::runtime_error(msg);
        }
        if (!transfer["out"]) {
          std::string msg = "ERROR: transfer n°" + std::to_string(safety_nets.size()) + " of net " +
            safety_net.name + " : definition without «out» parameter";
          RCLCPP_FATAL(
            rclcpp::get_logger("EthercatSafetyDriver"), msg.c_str());
          throw std::runtime_error(msg);
        }
        transfer_entry.size = transfer["size"].as<size_t>();
        getTransferMemoryInfo(
          transfer["in"], transfer_entry.input,
          "in", safety_net.name);
        getTransferMemoryInfo(
          transfer["out"], transfer_entry.output,
          "out", safety_net.name);
        safety_net.transfers.push_back(transfer_entry);
      }
      safety_nets.push_back(safety_net);
    }
  }

  return safety_nets;
}

void throwErrorIfModuleParametersNotFound(
  const ethercat_interface::EcTransferEntry & transfer,
  const std::string & module_name,
  const std::string & safety_net_name,
  const std::string & direction)
{
  std::string msg = "In safety net: " + safety_net_name + ", for transfer " +
    transfer.to_simple_string() + ", the module name of the " + direction + "( " + module_name +
    ") among all the recorded modules.";
  RCLCPP_ERROR(
    rclcpp::get_logger(
      "EthercatSafetyDriver"), msg.c_str());
  throw std::runtime_error(msg);
}

CallbackReturn CLASSM::on_init(
  const hardware_interface::HardwareInfo & info)
{
  auto res = this->EthercatDriver::on_init(info);
  if (res != CallbackReturn::SUCCESS) {
    return res;
  }

  // Prevent the same module from being activated while being configured
  const std::lock_guard<std::mutex> lock(ec_mutex_);
  activated_ = false;

  YAML::Node config;
  // Load the fsoe_config file
  loadFsoeConfigYamlFile(config);

  // Parse safety modules from the safety yaml file defined
  // in fsoe_config attribute of a param tag of the the URDF
  auto safety_module_params = getEcSafetyModuleParam(config);

  // Append the safety modules parameters to the list of modules parameters
  size_t idx_1st = ec_module_parameters_.size();
  ec_module_parameters_.insert(
    ec_module_parameters_.end(), safety_module_params.begin(), safety_module_params.end());
  for (size_t i = 0; i < safety_module_params.size(); i++) {
    ec_safety_slaves_.push_back(idx_1st + i);
  }

  // Parse safety nets from the safety yaml file defined
  // in fsoe_config attribute of a param tag of the the URDF
  ec_safety_nets_ = getEcSafetyNets(config);

  // Append the safety modules to the list of modules and load them
  for (const auto & safety_module_param : safety_module_params) {
    try {
      auto ec_module = ec_loader_.createSharedInstance(safety_module_param.at("plugin"));
      if (!ec_module->setupSlave(
          safety_module_param, &empty_interface_, &empty_interface_))
      {
        const std::string & module_name = safety_module_param.at("name");
        RCLCPP_FATAL(
          rclcpp::get_logger("EthercatDriver"),
          "Setup of safety only module %s FAILED.", module_name.c_str() );
        return CallbackReturn::ERROR;
      }

      auto idx = ec_modules_.size();
      ec_module->setAliasAndPosition(
        getAliasOrDefaultAlias(safety_module_param),
        std::stoul(safety_module_param.at("position")));
      ec_modules_.push_back(ec_module);
      ec_safety_slaves_.push_back(idx);
    } catch (const pluginlib::PluginlibException & ex) {
      const std::string & module_name = safety_module_param.at("name");
      RCLCPP_ERROR(
        rclcpp::get_logger(
          "EthercatSafetyDriver"),
        "The plugin failed to load for safety module %s. Error: %s\n",
        module_name.c_str(), ex.what());
    }
  }

  // Find all masters from the nets
  {
    std::vector<std::string> master_names;
    for (const auto & net : ec_safety_nets_) {
      master_names.push_back(net.master);
    }
    for (size_t i = 0; i < ec_module_parameters_.size(); i++) {
      if (std::find(
          master_names.begin(), master_names.end(),
          ec_module_parameters_[i].at("name")) !=
        master_names.end())
      {
        ec_safety_masters_.push_back(i);
      }
    }
  }

  // Identify (alias,position) all the modules participating in transfers
  for (auto & net : ec_safety_nets_) {
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

  return CallbackReturn::SUCCESS;
}

CallbackReturn CLASSM::setupMaster()
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

  // Safety master
  safety_ = std::make_shared<ethercat_interface::EcSafety>(master_id);
  master_ = safety_;

  return CallbackReturn::SUCCESS;
}

CallbackReturn CLASSM::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const std::lock_guard<std::mutex> lock(ec_mutex_);
  if (activated_) {
    RCLCPP_FATAL(rclcpp::get_logger("EthercatSafetyDriver"), "Double on_activate()");
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("EthercatSafetyDriver"), "Starting ...please wait...");

  // Setup master
  setupMaster();

  // Standard Network configuration
  configNetwork();

  if (!master_->activate()) {
    RCLCPP_ERROR(rclcpp::get_logger("EthercatSafetyDriver"), "Activate EcSafety failed");
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("EthercatSafetyDriver"), "Activated EcSafety!");

  // Safety Network configuration
  safety_->registerTransferInDomain(ec_safety_nets_);

  // start after one second
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  t.tv_sec++;

  bool running = true;
  while (running) {
    // wait until next shot
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
    // update EtherCAT bus

    master_->update(rclcpp::get_logger("EthercatSafetyDriver"));
    RCLCPP_INFO(rclcpp::get_logger("EthercatSafetyDriver"), "updated!");

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
    rclcpp::get_logger("EthercatSafetyDriver"), "System Successfully started!");

  activated_ = true;

  return CallbackReturn::SUCCESS;
}

}  // namespace ethercat_driver

#undef CLASSM

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ethercat_driver::EthercatSafetyDriver, hardware_interface::SystemInterface)
