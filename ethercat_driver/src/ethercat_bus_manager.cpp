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

#include "ethercat_driver/ethercat_bus_manager.hpp"

#include <algorithm>
#include <ctime>
#include <regex>
#include <stdexcept>
#include <string>

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
      "EthercatBusManager"), msg.c_str());
  throw std::runtime_error(msg);
}

uint16_t EthercatBusManager::getAliasOrDefaultAlias(
  const std::unordered_map<std::string,
  std::string> & slave_parameters)
{
  if (slave_parameters.find("alias") != slave_parameters.end()) {
    return std::stoul(slave_parameters.at("alias"));
  } else {
    return 0;
  }
}

bool EthercatBusManager::configureModules(
  const EthercatBusConfig & bus_config,
  const std::vector<ConfiguredEcModule> & modules)
{
  const std::lock_guard<std::mutex> lock(ec_mutex_);
  // configureModules must be called only when the bus is not active.
  // ros2_control's lifecycle guarantees this for on_init(); reject otherwise
  // because clearing internal state below would orphan an active master_
  // without going through stop().
  if (activated_) {
    RCLCPP_FATAL(
      rclcpp::get_logger("EthercatBusManager"),
      "configureModules() called while bus is active; deactivate first.");
    return false;
  }
  bus_config_ = bus_config;
  ec_modules_.clear();
  ec_module_parameters_.clear();
  ec_transfer_nets_.clear();
  ec_transfer_masters_.clear();
  ec_transfer_slaves_.clear();
  master_.reset();
  configured_ = false;

  // Collect all module parameters up front so the load loop mirrors the
  // original on_init() body structure (plugin load + setupSlave + push to ec_modules_).
  ec_module_parameters_.reserve(modules.size());
  for (const auto & configured_module : modules) {
    ec_module_parameters_.push_back(configured_module.parameters);
  }

  for (const auto & configured_module : modules) {
    try {
      auto module = ec_loader_.createSharedInstance(configured_module.parameters.at("plugin"));
      if (!module->setupSlave(
          configured_module.parameters,
          configured_module.input_values,
          configured_module.output_values))
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("EthercatBusManager"),
          "Setup of %s module %zu FAILED.",
          configured_module.module_type.c_str(),
          configured_module.module_number);
        return false;
      }
      module->setAliasAndPosition(
        getAliasOrDefaultAlias(configured_module.parameters),
        std::stoul(configured_module.parameters.at("position")));
      ec_modules_.push_back(module);
    } catch (pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        rclcpp::get_logger("EthercatBusManager"),
        "The plugin of %s failed to load for some reason. Error: %s\n",
        configured_module.component_name.c_str(), ex.what());
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("EthercatBusManager"), "Got %li modules", ec_modules_.size());

  // Check if a transfer configuration is provided
  if (!bus_config_.fsoe_config.empty() || !bus_config_.transfer_config.empty()) {
    RCLCPP_INFO(rclcpp::get_logger("EthercatBusManager"), "Transfer configuration detected, ...");

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
            rclcpp::get_logger("EthercatBusManager"),
            "Setup of transfer only module %s FAILED.", module_name.c_str() );
          return false;
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
            "EthercatBusManager"),
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
      rclcpp::get_logger("EthercatBusManager"),
      "Transfer configuration loaded successfully!");
  }

  return true;
}

bool EthercatBusManager::setupMaster()
{
  master_ = std::make_shared<ethercat_interface::EcMaster>(bus_config_.master_id);

  // ecrt_request_master() can fail (master not running, or /dev/EtherCATx not
  // accessible to this process). The EcMaster ctor only warns in that case and
  // leaves a null master handle.
  if (!master_ || !master_->isValid()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("EthercatBusManager"),
      "Failed to obtain EtherCAT master %u. Is the master running and is "
      "/dev/EtherCAT%u accessible to this process (permissions)?",
      bus_config_.master_id, bus_config_.master_id);
    master_.reset();
    return false;
  }

  return true;
}

bool EthercatBusManager::configNetwork()
{
  // start EC and wait until state operative

  control_frequency_ = bus_config_.control_frequency;
  master_->setCtrlFrequency(control_frequency_);

  for (auto i = 0ul; i < ec_modules_.size(); i++) {
    if (!master_->addSlave(ec_modules_[i].get())) {
      RCLCPP_ERROR(
        rclcpp::get_logger("EthercatBusManager"),
        "Failed to add slave for module at position %s; refusing to configure the bus. "
        "Check the drive is powered and present on the bus and that the slave_config "
        "matches the hardware (see the EcMaster error above for the exact cause).",
        ec_module_parameters_[i]["position"].c_str());
      return false;
    }
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
        RCLCPP_ERROR(
          rclcpp::get_logger("EthercatBusManager"),
          "Failed to download config SDO for module at position %s with Error: %d",
          ec_module_parameters_[i]["position"].c_str(),
          abort_code);
      }
    }
  }

  return true;
}

bool EthercatBusManager::configureBus()
{
  const std::lock_guard<std::mutex> lock(ec_mutex_);
  return configureBusLocked();
}

bool EthercatBusManager::configureBusLocked()
{
  if (activated_) {
    RCLCPP_FATAL(
      rclcpp::get_logger("EthercatBusManager"), "configureBus() called while active.");
    return false;
  }
  if (configured_) {
    return true;  // idempotent: master already requested and network configured
  }

  // setup master
  if (!setupMaster()) {
    return false;
  }
  // configure network (leaves the bus in the idle/PRE-OP phase)
  if (!configNetwork()) {
    return false;
  }
  configured_ = true;
  return true;
}

bool EthercatBusManager::activateBus()
{
  const std::lock_guard<std::mutex> lock(ec_mutex_);
  // Configure first if a caller skipped the explicit configureBus() step (keeps
  // the original single-call contract for existing consumers like EthercatDriver).
  if (!configured_) {
    if (!configureBusLocked()) {
      return false;
    }
  }
  return activateBusLocked();
}

bool EthercatBusManager::activateBusLocked()
{
  if (activated_) {
    RCLCPP_FATAL(rclcpp::get_logger("EthercatBusManager"), "Double on_activate()");
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("EthercatBusManager"), "Starting ...please wait...");

  if (!master_->activate()) {
    RCLCPP_ERROR(rclcpp::get_logger("EthercatBusManager"), "Activate EcMaster failed");
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("EthercatBusManager"), "Activated EcMaster!");

  // Configure transfer network if transfer nets are defined
  if (!ec_transfer_nets_.empty()) {
    RCLCPP_INFO(rclcpp::get_logger("EthercatBusManager"), "Configuring transfer network...");
    master_->registerTransferInDomain(ec_transfer_nets_);
    RCLCPP_INFO(rclcpp::get_logger("EthercatBusManager"), "Transfer network configured!");
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
    rclcpp::get_logger("EthercatBusManager"), "System Successfully started!");

  activated_ = true;

  return true;
}

void EthercatBusManager::deactivateBus()
{
  const std::lock_guard<std::mutex> lock(ec_mutex_);
  activated_ = false;

  RCLCPP_INFO(rclcpp::get_logger("EthercatBusManager"), "Stopping ...please wait...");

  // stop EC and disconnect
  master_->stop();

  RCLCPP_INFO(
    rclcpp::get_logger("EthercatBusManager"), "System successfully stopped!");
}

EthercatCycleResult EthercatBusManager::read()
{
  // try to lock so we can avoid blocking the read/write loop on the lock.
  const std::unique_lock<std::mutex> lock(ec_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    return EthercatCycleResult::kSkippedBusy;
  }
  if (activated_) {
    master_->readData();
    return EthercatCycleResult::kCompleted;
  }
  return EthercatCycleResult::kSkippedInactive;
}

EthercatCycleResult EthercatBusManager::write()
{
  // try to lock so we can avoid blocking the read/write loop on the lock.
  const std::unique_lock<std::mutex> lock(ec_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    return EthercatCycleResult::kSkippedBusy;
  }
  if (activated_) {
    master_->writeData();
    return EthercatCycleResult::kCompleted;
  }
  return EthercatCycleResult::kSkippedInactive;
}

int EthercatBusManager::readSlaveSdo(
  uint16_t slave_position, uint16_t index, uint8_t sub_index,
  uint8_t * target, size_t target_size, size_t * result_size, uint32_t * abort_code)
{
  const std::lock_guard<std::mutex> lock(ec_mutex_);
  if (!master_) {
    return -1;
  }
  return master_->uploadSlaveSdo(
    slave_position, index, sub_index, target, target_size, result_size, abort_code);
}

void EthercatBusManager::loadTransferConfigYamlFile(YAML::Node & node, const std::string & path)
{
  std::string file_path;
  if (path.empty()) {
    // Get the fsoe_config or transfer_config path from the bus configuration
    if (bus_config_.fsoe_config.empty() && bus_config_.transfer_config.empty()) {
      std::string msg("transfer_config or fsoe_config parameter is missing!");
      // Transfer (or fsoe) config file was not provided
      RCLCPP_FATAL(
        rclcpp::get_logger("EthercatBusManager"), msg.c_str());
      throw std::runtime_error(msg);
    }
    if (!bus_config_.fsoe_config.empty() && !bus_config_.transfer_config.empty()) {
      std::string msg(
        "Both transfer_config and fsoe_config parameters are provided! Please provide only one "
        "of them.");
      RCLCPP_FATAL(
        rclcpp::get_logger("EthercatBusManager"), msg.c_str());
      throw std::runtime_error(msg);
    }
    if (!bus_config_.fsoe_config.empty()) {
      std::string msg("The fsoe_config parameter is deprecated. "
        "Please use transfer_config instead.");
      RCLCPP_WARN(
        rclcpp::get_logger("EthercatBusManager"), msg.c_str());
      file_path = bus_config_.fsoe_config;
    }
    if (!bus_config_.transfer_config.empty()) {
      file_path = bus_config_.transfer_config;
    }
  } else {
    file_path = path;
  }

  try {
    node = YAML::LoadFile(file_path);
  } catch (const YAML::ParserException & ex) {
    std::string msg =
      std::string(
      "EthercatBusManager : failed to load transfer configuration "
      "(YAML file is incorrect): ") + std::string(ex.what());
    RCLCPP_FATAL(
      rclcpp::get_logger("EthercatBusManager"), msg.c_str() );
    throw std::runtime_error(msg);
  } catch (const YAML::BadFile & ex) {
    std::string msg =
      std::string(
      "EthercatBusManager : failed to load transfer configuration "
      "(file path is incorrect or file is damaged): " + std::string(ex.what()));
    RCLCPP_FATAL(
      rclcpp::get_logger("EthercatBusManager"), msg.c_str() );
    throw std::runtime_error(msg);
  } catch (std::exception & e) {
    std::string msg =
      std::string(
      "EthercatBusManager : error while loading transfer configuration: ") + std::string(e.what());
    RCLCPP_FATAL(
      rclcpp::get_logger("EthercatBusManager"), msg.c_str() );
    throw std::runtime_error(msg);
  }
}

std::vector<std::unordered_map<std::string,
  std::string>> EthercatBusManager::getEcTransferModuleParam(
  const YAML::Node & config)
{
  if (0 == config.size() ) {
    std::string msg = "Empty transfer_config or fsoe_config parameter!";
    RCLCPP_FATAL(
      rclcpp::get_logger("EthercatBusManager"), msg.c_str());
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

std::vector<ethercat_interface::EcTransferNet> EthercatBusManager::getEcTransferNets(
  const YAML::Node & config)
{
  if (0 == config.size() ) {
    std::string msg = "Empty transfer_config or fsoe_config parameter!";
    RCLCPP_FATAL(
      rclcpp::get_logger("EthercatBusManager"), msg.c_str());
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
            rclcpp::get_logger("EthercatBusManager"), msg.c_str());
          throw std::runtime_error(msg);
        }
        if (!transfer["in"]) {
          std::string msg = "ERROR: transfer n°" + std::to_string(transfer_nets.size()) +
            " of net " +
            transfer_net.name + " : definition without «in» parameter";
          RCLCPP_FATAL(
            rclcpp::get_logger("EthercatBusManager"), msg.c_str());
          throw std::runtime_error(msg);
        }
        if (!transfer["out"]) {
          std::string msg = "ERROR: transfer n°" + std::to_string(transfer_nets.size()) +
            " of net " +
            transfer_net.name + " : definition without «out» parameter";
          RCLCPP_FATAL(
            rclcpp::get_logger("EthercatBusManager"), msg.c_str());
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

}  // namespace ethercat_driver
