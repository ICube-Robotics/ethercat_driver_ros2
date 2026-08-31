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
#include <chrono>
#include <cstdio>
#include <ctime>
#include <regex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ethercat_driver
{
namespace
{
std::string wc_state_str(uint8_t wc_state)
{
  switch (wc_state) {
    case 0: return "ZERO";
    case 1: return "INCOMPLETE";
    case 2: return "COMPLETE";
    default: return "UNKNOWN";
  }
}

std::string al_states_mask_str(uint8_t al_states)
{
  if (al_states == 0) {return "NONE";}
  static constexpr std::pair<uint8_t, const char *> kBits[] = {
    {1, "INIT"}, {2, "PREOP"}, {4, "SAFEOP"}, {8, "OP"}};
  std::string out;
  uint8_t seen = 0;
  for (const auto & [bit, name] : kBits) {
    if (al_states & bit) {
      if (!out.empty()) {out += "|";}
      out += name;
      seen |= bit;
    }
  }
  if (const uint8_t stray = al_states & static_cast<uint8_t>(~seen); stray != 0) {
    char buf[16];
    std::snprintf(buf, sizeof(buf), "UNKNOWN(0x%X)", stray);
    if (!out.empty()) {out += "|";}
    out += buf;
  }
  return out;
}

uint8_t diagnostic_level_for_bus(bool link_up, uint8_t domain_wc_state)
{
  if (!link_up) {return diagnostic_msgs::msg::DiagnosticStatus::ERROR;}
  if (domain_wc_state != 2 /* COMPLETE */) {return diagnostic_msgs::msg::DiagnosticStatus::WARN;}
  return diagnostic_msgs::msg::DiagnosticStatus::OK;
}
}  // namespace

pluginlib::ClassLoader<ethercat_interface::EcMasterBase>
EthercatBusManager::ec_master_loader_{"ethercat_interface", "ethercat_interface::EcMasterBase"};
pluginlib::ClassLoader<ethercat_interface::EcSlaveBase>
EthercatBusManager::ec_slave_loader_{"ethercat_interface", "ethercat_interface::EcSlaveBase"};


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

bool EthercatBusManager::getRequiredOrDefault(
  const std::unordered_map<std::string,
  std::string> & slave_parameters)
{
  const auto it = slave_parameters.find("required");
  // Absent (the overwhelmingly common case today) or "true": today's behavior, unchanged —
  // a failing module refuses to configure the whole bus.
  return it == slave_parameters.end() || it->second != "false";
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
      auto module =
        ec_slave_loader_.createSharedInstance(configured_module.parameters.at("plugin"));
      if (!module->setup_slave(
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
        auto ec_module = ec_slave_loader_.createSharedInstance(transfer_module_param.at("plugin"));
        if (!ec_module->setup_slave(
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

        transfer.input.alias = input_module->get_alias();
        transfer.input.position = input_module->get_position();
        transfer.output.alias = output_module->get_alias();
        transfer.output.position = output_module->get_position();
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
  // Dynamically load master plugin
  try {
    master_ = ec_master_loader_.createSharedInstance(bus_config_.master_plugin);
  } catch (pluginlib::PluginlibException & ex) {
    RCLCPP_FATAL(
      rclcpp::get_logger("EthercatDriver"),
      "The master plugin %s failed to load for some reason. Error: %s\n",
      bus_config_.master_plugin.c_str(), ex.what());
    return false;
  }
  if (!master_->init(bus_config_.master_iface)) {
    RCLCPP_FATAL(
      rclcpp::get_logger("EthercatDriver"),
      "Failed to initialize Master. Aborting.");
    return false;
  }

  // The master plugin's init() can fail (master not running, or /dev/EtherCATx not
  // accessible to this process); is_valid() is a defense-in-depth check in case a plugin
  // returns true from init() but still leaves an invalid internal master handle.
  if (!master_ || !master_->is_valid()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("EthercatBusManager"),
      "Failed to obtain EtherCAT master '%s'. Is the master running and is "
      "/dev/EtherCAT%s accessible to this process (permissions)?",
      bus_config_.master_iface.c_str(), bus_config_.master_iface.c_str());
    master_.reset();
    return false;
  }

  return true;
}

bool EthercatBusManager::registerSlaves()
{
  // start EC and wait until state operative

  control_frequency_ = bus_config_.control_frequency;
  master_->set_ctrl_frequency(control_frequency_);

  // Phase 1: validate every module (identity + sdo_check:) with no side effects, before any
  // module is registered — both checks are addressed by ring position at the master level and
  // need no prior slave configuration. A module that fails and is required (absent "required"
  // param, or "required: true" — today's default and only behavior) refuses to configure the
  // whole bus, exactly as before. A module explicitly marked "required: false" is excluded
  // instead: it's simply never added in phase 2 below, so it's absent from the cyclic domain
  // and reads as offline (EthercatClient::slaveHealth() / motor_drive_controller's existing
  // slave_online handling already treats "no matching entry in get_slave_states()" as offline
  // — no new state or interface needed for this).
  std::vector<bool> module_ok(ec_modules_.size(), false);
  for (auto i = 0ul; i < ec_modules_.size(); i++) {
    module_ok[i] = master_->check_slave(ec_modules_[i]);
    if (!module_ok[i]) {
      if (getRequiredOrDefault(ec_module_parameters_[i])) {
        RCLCPP_ERROR(
          rclcpp::get_logger("EthercatBusManager"),
          "Module at position %s failed validation and is required (the default); refusing "
          "to configure the bus (see the master plugin's error above for the exact cause). "
          "Mark it 'required: false' in the URDF to let the rest of the bus come up without "
          "it instead.",
          ec_module_parameters_[i]["position"].c_str());
        return false;
      }
      RCLCPP_WARN(
        rclcpp::get_logger("EthercatBusManager"),
        "Module at position %s failed validation and is marked 'required: false'; excluding "
        "it from the bus — it will not be readable or writable (see the master plugin's "
        "error above for the exact cause). The rest of the bus will still configure.",
        ec_module_parameters_[i]["position"].c_str());
    }
  }

  // Phase 2: register (PDO/domain mapping) only the modules that passed phase 1.
  for (auto i = 0ul; i < ec_modules_.size(); i++) {
    if (!module_ok[i]) {continue;}
    if (!master_->add_slave(ec_modules_[i])) {
      RCLCPP_ERROR(
        rclcpp::get_logger("EthercatBusManager"),
        "Failed to add slave for module at position %s; refusing to configure the bus. "
        "Check the drive is powered and present on the bus and that the slave_config "
        "matches the hardware (see the master plugin's error above for the exact cause).",
        ec_module_parameters_[i]["position"].c_str());
      return false;
    }
  }

  return true;
}

bool EthercatBusManager::downloadSdoConfig()
{
  RCLCPP_INFO(
    rclcpp::get_logger("EthercatBusManager"),
    "Downloading config SDO(s) to %zu module(s)...", ec_modules_.size());
  if (!master_->configure_slaves()) {
    RCLCPP_FATAL(
      rclcpp::get_logger("EthercatDriver"),
      "Failed to configure Slaves. Aborting.");
    return false;
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
    return true;  // idempotent: SDO config already downloaded once for this master
  }

  // Request the master at most once for the lifetime of this EthercatBusManager: a
  // deactivate -> reactivate cycle must rebuild the PDO/domain registration (see
  // activateBusLocked()), but must NOT re-request the master — gate on master_ existing,
  // not on configured_.
  if (!master_ && !setupMaster()) {
    return false;
  }
  // Register slaves + PDO domain, then download the one-shot config SDOs. Both leave the
  // bus in the idle/PRE-OP phase (not yet activated).
  if (!registerSlaves()) {
    return false;
  }
  if (!downloadSdoConfig()) {
    return false;
  }
  network_registered_ = true;
  configured_ = true;
  return true;
}

bool EthercatBusManager::activateBus()
{
  {
    const std::lock_guard<std::mutex> lock(ec_mutex_);
    // Configure first if a caller skipped the explicit configureBus() step (keeps
    // the original single-call contract for existing consumers like EthercatDriver).
    if (!configured_) {
      if (!configureBusLocked()) {
        return false;
      }
    }
    if (!activateBusLocked()) {
      return false;
    }
  }  // Lock released: activated_ is now true, so read()/write() are live - required by
     // waitForSlavesOperational(), which calls them and would otherwise always observe
     // this same thread already holding ec_mutex_.

  if (!waitForSlavesOperational()) {
    deactivateBus();
    return false;
  }
  return true;
}

bool EthercatBusManager::activateBusLocked()
{
  if (activated_) {
    RCLCPP_FATAL(rclcpp::get_logger("EthercatBusManager"), "Double on_activate()");
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("EthercatBusManager"), "Starting ...please wait...");

  // On a reactivation (after deactivateBus()), EcMasterBase::deactivate() has freed the
  // ecrt-side PDO/domain registration — rebuild it here. Deliberately NOT
  // downloadSdoConfig(): those are one-shot config SDOs already sent once in
  // configureBusLocked() and must not be resent (they are not cyclic PDO data).
  if (!network_registered_) {
    if (!registerSlaves()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("EthercatBusManager"),
        "Failed to re-register slaves/PDO domain on reactivation.");
      return false;
    }
    network_registered_ = true;
  }

  if (!master_->start()) {
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

  // Deliberately NOT waiting here for slaves to reach OPERATIONAL: this method runs with
  // ec_mutex_ held, and that wait needs read()/write() (see waitForSlavesOperational()),
  // which take their own try_to_lock and would see this thread already holding the lock on
  // every attempt. activateBus() runs the actual bounded wait after releasing this lock.
  RCLCPP_INFO(
      rclcpp::get_logger("EthercatDriver"),
      "EcMaster active. Waiting for slaves to reach OPERATIONAL...");

  activated_ = true;

  return true;
}

bool EthercatBusManager::waitForSlavesOperational()
{
  const auto period = std::chrono::duration<double>(1.0 / control_frequency_);
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(bus_config_.readiness_timeout_s));

  while (std::chrono::steady_clock::now() < deadline) {
    if (read() == EthercatCycleResult::kError) {
      RCLCPP_ERROR(rclcpp::get_logger("EthercatBusManager"),
        "waitForSlavesOperational(): read() failed during settle.");
      return false;
    }
    if (write() == EthercatCycleResult::kError) {
      RCLCPP_ERROR(rclcpp::get_logger("EthercatBusManager"),
        "waitForSlavesOperational(): write() failed during settle.");
      return false;
    }

    bool all_operational = true;
    for (const auto & slave : slaveStates()) {
      if (!slave.operational) {
        all_operational = false;
        break;
      }
    }
    if (all_operational) {
      RCLCPP_INFO(
        rclcpp::get_logger("EthercatBusManager"), "All slaves reached OPERATIONAL.");
      return true;
    }

    std::this_thread::sleep_for(
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(period));
  }

  RCLCPP_ERROR(rclcpp::get_logger("EthercatBusManager"),
    "Timed out (%.1fs) waiting for all slaves to reach OPERATIONAL.",
    bus_config_.readiness_timeout_s);
  return false;
}

void EthercatBusManager::deactivateBus()
{
  const std::lock_guard<std::mutex> lock(ec_mutex_);
  activated_ = false;

  RCLCPP_INFO(rclcpp::get_logger("EthercatBusManager"), "Stopping ...please wait...");

  // Stop our own cyclic send/receive, then formally deactivate the EtherCAT master (per spec,
  // this is what makes the slaves' Sync Manager Watchdog drop them OP -> Safe-OP instead of
  // leaving them stuck at OP with a dead master indefinitely). This frees the domain/slave-config
  // objects the master plugin holds, so a subsequent activation must re-register the network —
  // the master reservation itself (master_) is untouched and stays valid for reuse.
  master_->stop();
  if (!master_->deactivate()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("EthercatBusManager"), "Failed to deactivate EtherCAT master");
  }
  // The PDO/domain registration deactivate() just freed must be rebuilt before the next
  // activate() (see activateBusLocked()). configured_ stays true: the config SDOs already
  // downloaded must not be resent on reactivation.
  network_registered_ = false;

  RCLCPP_INFO(
    rclcpp::get_logger("EthercatBusManager"), "System successfully stopped!");
}

void EthercatBusManager::setDiagnosticsNode(rclcpp::Node::SharedPtr node)
{
  diagnostics_node_ = node;
  bus_diagnostics_publisher_ =
    diagnostics_node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", rclcpp::SystemDefaultsQoS());
  rt_bus_diagnostics_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>>(
    bus_diagnostics_publisher_);

  slave_diagnostics_publisher_ =
    diagnostics_node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", rclcpp::SystemDefaultsQoS());
  rt_slave_diagnostics_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>>(
    slave_diagnostics_publisher_);
}

void EthercatBusManager::publishBusDiagnostics()
{
  if (!rt_bus_diagnostics_publisher_) {
    return;
  }
  const auto time = diagnostics_node_->now();

  const ethercat_interface::EcMasterStateInfo master_state = masterState();
  const ethercat_interface::EcDomainStateInfo domain_state = domainState();
  const std::string current_bus_state = (master_state.link_up ? "UP" : "DOWN") + std::string(":") +
    wc_state_str(domain_state.wc_state);
  const bool state_changed = current_bus_state != previous_bus_state_;
  const bool heartbeat_due = !bus_diagnostics_publish_time_valid_ ||
    (time - last_bus_diagnostics_publish_time_) >= rclcpp::Duration::from_seconds(1.0);

  if ((state_changed || heartbeat_due) && rt_bus_diagnostics_publisher_->trylock()) {
    auto & msg = rt_bus_diagnostics_publisher_->msg_;
    msg.header.stamp = time;
    msg.status.resize(1);

    auto & status = msg.status[0];
    status.name = "ethercat_bus_manager";
    status.hardware_id = bus_config_.master_iface;
    status.level = diagnostic_level_for_bus(master_state.link_up, domain_state.wc_state);
    status.message = master_state.link_up ?
      ("bus link up, domain " + wc_state_str(domain_state.wc_state)) :
      "bus link DOWN";
    status.values.clear();
    status.values.reserve(4);
    {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = "slaves_responding";
      kv.value = std::to_string(master_state.slaves_responding);
      status.values.push_back(std::move(kv));
    }
    {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = "al_states";
      kv.value = al_states_mask_str(master_state.al_states);
      status.values.push_back(std::move(kv));
    }
    {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = "domain_working_counter";
      kv.value = std::to_string(domain_state.working_counter);
      status.values.push_back(std::move(kv));
    }
    {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = "domain_wc_state";
      kv.value = wc_state_str(domain_state.wc_state);
      status.values.push_back(std::move(kv));
    }

    rt_bus_diagnostics_publisher_->unlockAndPublish();
    last_bus_diagnostics_publish_time_ = time;
    bus_diagnostics_publish_time_valid_ = true;
  }

  previous_bus_state_ = current_bus_state;
}

void EthercatBusManager::publishSlaveDiagnostics()
{
  if (ec_modules_.empty()) {
    return;
  }

  // A slave has no visibility into its own online/al_state; push this cycle's observation in,
  // correlated by alias+position (matching the master's own addressing).
  const std::vector<ethercat_interface::EcSlaveStateInfo> states = slaveStates();
  for (const auto & module : ec_modules_) {
    bool found = false;
    for (const auto & s : states) {
      if (s.alias == module->get_alias() && s.position == module->get_position()) {
        module->setHealth(s.online, s.al_state);
        found = true;
        break;
      }
    }
    if (!found) {
      module->setHealth(false, 0);
    }
  }

  if (!rt_slave_diagnostics_publisher_) {
    return;
  }
  const auto time = diagnostics_node_->now();

  std::vector<std::string> current_slave_states(ec_modules_.size());
  bool state_changed = previous_slave_states_.size() != ec_modules_.size();
  for (std::size_t i = 0; i < ec_modules_.size(); ++i) {
    diagnostic_msgs::msg::DiagnosticStatus probe;
    ec_modules_[i]->collectDiagnostics(probe);
    current_slave_states[i] = std::to_string(probe.level) + ":" + probe.message;
    if (!state_changed &&
      (previous_slave_states_.empty() || current_slave_states[i] != previous_slave_states_[i]))
    {
      state_changed = true;
    }
  }

  const bool heartbeat_due = !slave_diagnostics_publish_time_valid_ ||
    (time - last_slave_diagnostics_publish_time_) >= rclcpp::Duration::from_seconds(1.0);

  if ((state_changed || heartbeat_due) && rt_slave_diagnostics_publisher_->trylock()) {
    auto & msg = rt_slave_diagnostics_publisher_->msg_;
    msg.header.stamp = time;
    msg.status.resize(ec_modules_.size());
    for (std::size_t i = 0; i < ec_modules_.size(); ++i) {
      auto & status = msg.status[i];
      const auto name_it = ec_module_parameters_[i].find("name");
      status.name = name_it != ec_module_parameters_[i].end() ? name_it->second : "slave";
      status.hardware_id = status.name;
      status.values.clear();
      ec_modules_[i]->collectDiagnostics(status);
    }
    rt_slave_diagnostics_publisher_->unlockAndPublish();
    last_slave_diagnostics_publish_time_ = time;
    slave_diagnostics_publish_time_valid_ = true;
  }

  previous_slave_states_ = std::move(current_slave_states);
}

EthercatCycleResult EthercatBusManager::read()
{
  // try to lock so we can avoid blocking the read/write loop on the lock.
  const std::unique_lock<std::mutex> lock(ec_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    return EthercatCycleResult::kSkippedBusy;
  }
  if (activated_) {
    master_->read_process_data();
    publishBusDiagnostics();
    publishSlaveDiagnostics();
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
    master_->write_process_data();
    return EthercatCycleResult::kCompleted;
  }
  return EthercatCycleResult::kSkippedInactive;
}

ethercat_interface::EcMasterStateInfo EthercatBusManager::masterState() const
{
  if (!master_) {return {};}
  return master_->get_master_state();
}

ethercat_interface::EcDomainStateInfo EthercatBusManager::domainState(uint32_t domain) const
{
  if (!master_) {return {};}
  return master_->get_domain_state(domain);
}

std::vector<ethercat_interface::EcSlaveStateInfo> EthercatBusManager::slaveStates() const
{
  if (!master_) {return {};}
  return master_->get_slave_states();
}

int EthercatBusManager::readSlaveSdo(
  uint16_t slave_position, uint16_t index, uint8_t sub_index,
  uint8_t * target, size_t target_size, size_t * result_size, uint32_t * abort_code,
  uint16_t alias)
{
  const std::lock_guard<std::mutex> lock(ec_mutex_);
  if (!master_) {
    return -1;
  }
  return master_->upload_slave_sdo(
    slave_position, index, sub_index, target, target_size, result_size, abort_code, alias);
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
