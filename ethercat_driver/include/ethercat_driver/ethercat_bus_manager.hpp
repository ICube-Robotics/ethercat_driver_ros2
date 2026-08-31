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

#include "ethercat_interface/ec_master_base.hpp"
#include "ethercat_interface/ec_slave_base.hpp"

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
  std::string master_iface{"0"};
  std::string master_plugin{"ethercat_master/EtherlabMaster"};
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
  EthercatBusManager() {}
  ~EthercatBusManager() {master_.reset(); ec_modules_.clear();}

  bool configureModules(
    const EthercatBusConfig & bus_config,
    const std::vector<ConfiguredEcModule> & modules);

  /** @brief Request the master and configure the network, leaving the bus in the
   * idle/PRE-OP phase (not yet activated). Idempotent. This is the phase in which
   * blocking SDO access (configSlaveSdo, readSlaveSdo) is valid — once the bus is
   * activated the application must drive the cyclic loop and blocking SDO calls
   * would stall. activateBus() calls this automatically if not already configured,
   * so existing callers are unaffected.
   */
  bool configureBus();

  bool activateBus();

  void deactivateBus();

  EthercatCycleResult read();

  EthercatCycleResult write();

  /** @brief Read a slave SDO entry (CoE upload). Must be called in the idle/PRE-OP
   * phase — i.e. after configureBus() but BEFORE activateBus(). This is a blocking
   * mailbox call; after the bus is activated the application owns the cyclic loop
   * and a blocking upload would stall the master (and the calling thread).
   * @return 0 on success, negative if the master does not exist, else the
   * ecrt_master_sdo_upload return code.
   */
  int readSlaveSdo(
    uint16_t slave_position, uint16_t index, uint8_t sub_index,
    uint8_t * target, size_t target_size, size_t * result_size, uint32_t * abort_code);

  /** @brief Bus-wide master state (link up/down, responding-slave count, aggregate AL
   *  states), as last observed by the master plugin's periodic check during read(). No new
   *  bus transaction — a zero-initialized struct if the master was never obtained.
   *  Not internally locked: intended to be called from the same thread immediately after
   *  read(), matching how read()/write() are used from the single ros2_control RT cycle. */
  ethercat_interface::EcMasterStateInfo masterState() const;

  /** @brief Domain (cyclic PDO exchange) state — working counter and completeness — as last
   *  observed by the master plugin during read(). No new bus transaction. Same threading
   *  note as masterState(). */
  ethercat_interface::EcDomainStateInfo domainState(uint32_t domain = 0) const;

  /** @brief Per-slave AL state / online / operational, as last observed by the master
   *  plugin's periodic check during read(). No new bus transaction. Same threading note as
   *  masterState(). */
  std::vector<ethercat_interface::EcSlaveStateInfo> slaveStates() const;

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

  /** Implementations of configureBus()/activateBus() that assume ec_mutex_ is
   * already held by the caller (avoids recursive locking when activateBus()
   * needs to configure first). */
  bool configureBusLocked();
  bool activateBusLocked();

protected:
  EthercatBusConfig bus_config_;
  std::vector<std::shared_ptr<ethercat_interface::EcSlaveBase>> ec_modules_;
  std::vector<std::unordered_map<std::string, std::string>> ec_module_parameters_;

  static pluginlib::ClassLoader<ethercat_interface::EcMasterBase> ec_master_loader_;
  static pluginlib::ClassLoader<ethercat_interface::EcSlaveBase> ec_slave_loader_;

  double control_frequency_;

  std::shared_ptr<ethercat_interface::EcMasterBase> master_;

  std::mutex ec_mutex_;
  bool configured_{false};
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
