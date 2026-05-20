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

#ifndef ETHERCAT_DRIVER__ETHERCAT_BUS_MANAGER_HPP_
#define ETHERCAT_DRIVER__ETHERCAT_BUS_MANAGER_HPP_

#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <pluginlib/class_loader.hpp>

#include "ethercat_interface/ec_master.hpp"
#include "ethercat_interface/ec_slave.hpp"
#include "yaml-cpp/yaml.h"

namespace ethercat_driver
{

struct ConfiguredEcModule
{
  std::unordered_map<std::string, std::string> parameters;
  std::vector<double> * input_values;
  std::vector<double> * output_values;
  std::string component_name;
  std::string module_type;
  size_t module_number;
};

/** Bus-wide EtherCAT settings independent of the ros2_control HardwareInfo representation. */
struct EthercatBusConfig
{
  unsigned int master_id{0};
  double control_frequency{100.0};
  std::string transfer_config;
  std::string fsoe_config;
};

enum class EthercatCycleResult
{
  kCompleted,
  kSkippedInactive,
  kSkippedBusy,
  kError
};

class EthercatBusManager
{
public:
  bool configureModules(
    const EthercatBusConfig & bus_config,
    const std::vector<ConfiguredEcModule> & modules);

  bool activateBus();

  void deactivateBus();

  EthercatCycleResult read();

  EthercatCycleResult write();

  /** @brief Get transfer module parameters from YAML file
   * @param[in] config YAML node containing the transfer configuration root
   * @return Vector of maps containing transfer module parameters, each map corresponds to a module
   * involved in a transfer
   */
  std::vector<std::unordered_map<std::string, std::string>> getEcTransferModuleParam(
    const YAML::Node & config);

  /** @brief Get transfer nets from YAML file
   * @param[in] config YAML node containing the transfer configuration root
   * @return Vector of transfer nets
   */
  std::vector<ethercat_interface::EcTransferNet> getEcTransferNets(const YAML::Node & config);

protected:
  uint16_t getAliasOrDefaultAlias(
    const std::unordered_map<std::string,
    std::string> & slave_parameters);

  /** @brief Load transfer config YAML file
   * One use case is to load transfers for FailSafe Over EtherCAT Safety
   * @param[out] node YAML node containing the transfer configuration root
   * @param[in] path Path to the YAML file, if empty, the file is loaded from the *fsoe_config*
   * or *transfer_config* of the YAML document
   */
  void loadTransferConfigYamlFile(YAML::Node & node, const std::string & path = "");

  bool setupMaster();

  bool configNetwork();

protected:
  EthercatBusConfig bus_config_;
  std::vector<std::shared_ptr<ethercat_interface::EcSlave>> ec_modules_;
  std::vector<std::unordered_map<std::string, std::string>> ec_module_parameters_;

  pluginlib::ClassLoader<ethercat_interface::EcSlave> ec_loader_{
    "ethercat_interface", "ethercat_interface::EcSlave"};

  double control_frequency_;

  std::shared_ptr<ethercat_interface::EcMaster> master_;
  std::mutex ec_mutex_;
  bool activated_{false};

  /** Transfer nets */
  std::vector<ethercat_interface::EcTransferNet> ec_transfer_nets_;

  /** Indexes of modules inside ec_modules_ vector that are transfer masters */
  std::vector<size_t> ec_transfer_masters_;
  /** Indexes of modules inside ec_modules_ vector that are transfer slaves only */
  std::vector<size_t> ec_transfer_slaves_;

  /** Empty interfaces */
  std::vector<double> empty_interface_;
};

}  // namespace ethercat_driver

#endif  // ETHERCAT_DRIVER__ETHERCAT_BUS_MANAGER_HPP_
