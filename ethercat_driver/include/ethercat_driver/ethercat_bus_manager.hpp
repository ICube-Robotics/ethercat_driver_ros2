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

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_publisher.hpp"

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
  // Bound on how long activateBus() waits for every slave to reach OPERATIONAL (bus AL
  // state, not any vendor drive-state machine) before giving up. Matches the naming/default
  // convention of a settle-timeout already used elsewhere for the same kind of wait.
  double readiness_timeout_s{25.0};
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

  /** @brief Request the master, register/activate the network, then block (bounded by
   * bus_config_.readiness_timeout_s) until every slave reports OPERATIONAL from the bus's
   * own point of view (EcSlaveStateInfo::operational, via slaveStates()) - not merely that
   * the master accepted activation. Without this, a caller's first read() after this
   * returns could still see a slave in the PREOP/SAFEOP transient, with process data that
   * has not started exchanging yet. Returns false (and rolls back via deactivateBus()) on
   * timeout or on any read()/write() failure during the wait. */
  bool activateBus();

  void deactivateBus();

  EthercatCycleResult read();

  EthercatCycleResult write();

  /** @brief Read a slave SDO entry (CoE upload). Must be called in the idle/PRE-OP
   * phase — i.e. after configureBus() but BEFORE activateBus(). This is a blocking
   * mailbox call; after the bus is activated the application owns the cyclic loop
   * and a blocking upload would stall the master (and the calling thread). alias 0
   * addresses slave_position as an absolute ring position; a non-zero alias makes
   * slave_position relative to it.
   * @return 0 on success, negative if the master does not exist, else the
   * ecrt_master_sdo_upload return code.
   */
  int readSlaveSdo(
    uint16_t slave_position, uint16_t index, uint8_t sub_index,
    uint8_t * target, size_t target_size, size_t * result_size, uint32_t * abort_code,
    uint16_t alias = 0);

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

  /** @brief Give this bus manager a node to publish diagnostics from. Optional — with no node
   *  set, diagnostics are simply not published. Creates the bus-diagnostics publisher; call
   *  before activateBus() so the first cycle can publish. */
  void setDiagnosticsNode(rclcpp::Node::SharedPtr node);

protected:
  uint16_t getAliasOrDefaultAlias(
    const std::unordered_map<std::string,
    std::string> & slave_parameters);

  /** @brief Whether a module's absence/misconfiguration should refuse to configure the whole
   *  bus (true, the default when the "required" param is absent — today's behavior, unchanged)
   *  or just exclude that module while the rest of the bus still comes up (false). */
  bool getRequiredOrDefault(
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

  /** @brief Validate (phase 1) then register (phase 2) every module's slave with the master —
   *  identity/sdo_check: gate, then ecrt_master_slave_config()/PDO-domain registration for
   *  every module that passed. No SDO traffic. Must be safely re-runnable: deactivateBus()
   *  frees this registration (see EcMasterBase::deactivate()), so activateBusLocked() re-runs
   *  this alone (never downloadSdoConfig()) to rebuild it ahead of a reactivation. */
  bool registerSlaves();

  /** @brief Download each registered module's configured SDO entries (the slave_config YAML
   *  `sdo:` block) to the drive. Config SDOs, not cyclic PDO data — must be sent exactly once,
   *  in configureBusLocked() (i.e. on_configure()), and never repeated on a later
   *  reactivation. */
  bool downloadSdoConfig();

  /** Implementations of configureBus()/activateBus() that assume ec_mutex_ is
   * already held by the caller (avoids recursive locking when activateBus()
   * needs to configure first). */
  bool configureBusLocked();
  bool activateBusLocked();

  /** @brief Poll read()/write() (the ordinary cyclic exchange, not a raw sleep) until
   * slaveStates() reports every slave operational, or bus_config_.readiness_timeout_s
   * elapses. Deliberately called with ec_mutex_ NOT held (see activateBus()): read()/write()
   * take their own try_to_lock, so holding the lock across this call would make every
   * iteration observe kSkippedBusy and never actually progress. */
  bool waitForSlavesOperational();

  /** @brief Publish bus-wide diagnostics (link_up, slaves_responding, al_states,
   *  domain_working_counter, domain_wc_state) if a diagnostics node has been set.
   *  Heartbeat every second, or immediately on change. Realtime-safe (trylock()); a busy
   *  publisher just skips that cycle. No-op if setDiagnosticsNode() was never called. */
  void publishBusDiagnostics();

  /** @brief Push this cycle's per-slave online/al_state into each module (a slave has no
   *  visibility into its own — see EcSlaveBase::setHealth()), then publish one DiagnosticArray
   *  with one status per module, built from each module's own collectDiagnostics(). Same
   *  heartbeat/on-change/realtime-safety notes as publishBusDiagnostics(). */
  void publishSlaveDiagnostics();

protected:
  EthercatBusConfig bus_config_;
  std::vector<std::shared_ptr<ethercat_interface::EcSlaveBase>> ec_modules_;
  std::vector<std::unordered_map<std::string, std::string>> ec_module_parameters_;

  static pluginlib::ClassLoader<ethercat_interface::EcMasterBase> ec_master_loader_;
  static pluginlib::ClassLoader<ethercat_interface::EcSlaveBase> ec_slave_loader_;

  double control_frequency_;

  std::shared_ptr<ethercat_interface::EcMasterBase> master_;

  std::mutex ec_mutex_;
  // True once downloadSdoConfig() has run (config SDOs sent). Never cleared by
  // deactivateBus(): those are one-shot config writes and must not repeat on a
  // reactivation — see downloadSdoConfig()'s doc comment.
  bool configured_{false};
  // True while the ecrt-side PDO/domain registration built by registerSlaves() is valid.
  // deactivateBus() clears this (EcMasterBase::deactivate() frees that registration);
  // activateBusLocked() rebuilds it via registerSlaves() alone before activating.
  bool network_registered_{false};
  bool activated_{false};

  /** Transfer nets */
  std::vector<ethercat_interface::EcTransferNet> ec_transfer_nets_;

  /** Indexes of modules inside ec_modules_ vector that are transfer masters */
  std::vector<size_t> ec_transfer_masters_;
  /** Indexes of modules inside ec_modules_ vector that are transfer slaves only */
  std::vector<size_t> ec_transfer_slaves_;

  /** Empty interfaces */
  std::vector<double> empty_interface_;

  rclcpp::Node::SharedPtr diagnostics_node_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr bus_diagnostics_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>>
  rt_bus_diagnostics_publisher_;
  std::string previous_bus_state_;
  bool bus_diagnostics_publish_time_valid_{false};
  rclcpp::Time last_bus_diagnostics_publish_time_;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr slave_diagnostics_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>>
  rt_slave_diagnostics_publisher_;
  std::vector<std::string> previous_slave_states_;
  bool slave_diagnostics_publish_time_valid_{false};
  rclcpp::Time last_slave_diagnostics_publish_time_;
};

}  // namespace ethercat_driver

#endif  // ETHERCAT_DRIVER__ETHERCAT_BUS_MANAGER_HPP_
