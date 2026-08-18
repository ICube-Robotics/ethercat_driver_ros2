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

#ifndef ETHERCAT_INTERFACE__EC_SLAVE_BASE_HPP_
#define ETHERCAT_INTERFACE__EC_SLAVE_BASE_HPP_

#include <map>
#include <vector>
#include <unordered_map>
#include <iostream>
#include <cmath>
#include <string>

#include "ethercat_interface/ec_sdo_manager.hpp"
#include "ethercat_interface/ec_pdo_channel_manager.hpp"
#include "ethercat_interface/ec_sync_manager.hpp"
#include "ethercat_interface/ec_buffer_tools.h"

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"


namespace ethercat_interface
{

typedef struct
{
  uint16_t index;   /**< PDO index. */
  unsigned int n_entries;   /**< Number of PDO entries in \a entries to map. */
  PdoType pdo_type;
} pdo_info_t;

/** EtherCAT AL state bitmask (ETG.1000 encoding, matches EcMasterStateInfo::al_states and
 *  EcSlaveStateInfo::al_state): INIT=1, PREOP=2, SAFEOP=4, OP=8. */
constexpr uint8_t kAlStateInit = 1;
constexpr uint8_t kAlStatePreop = 2;
constexpr uint8_t kAlStateSafeop = 4;
constexpr uint8_t kAlStateOp = 8;

inline std::string alStateStr(uint8_t al_state)
{
  switch (al_state) {
    case kAlStateInit: return "INIT";
    case kAlStatePreop: return "PREOP";
    case kAlStateSafeop: return "SAFEOP";
    case kAlStateOp: return "OP";
    default: return "UNKNOWN";
  }
}

class EcSlaveBase
{
public:
  EcSlaveBase() {}
  ~EcSlaveBase() {}

  /** read or write data to the domain */
  virtual void process_data(int /* index */, uint8_t * /*domain_address*/) {}

  /** Assign activate DC synchronization. return activate word*/
  virtual int assign_activate_dc_sync() {return 0x00;}
  virtual bool initialized() {return true;}
  virtual void set_state_is_operational(bool value) {is_operational_ = value;}

  virtual void updateState() {}

  inline
  void setAliasAndPosition(uint16_t alias, uint16_t position)
  {
    alias_ = alias;
    position_ = position;
    is_alias_and_position_set_ = true;
  }

  inline
  bool isAliasAndPositionSet()
  {
    return is_alias_and_position_set_;
  }

  /** Push this cycle's link/AL-state observation in. A slave has no visibility into its own
   *  online/al_state otherwise — both are tracked by the master's periodic state check.
   *  Called once per cycle by whoever owns the master (e.g. EthercatBusManager), correlated
   *  by alias/position against EcMasterBase::get_slave_states(). */
  inline
  void setHealth(bool online, uint8_t al_state)
  {
    online_ = online;
    al_state_ = al_state;
  }

  /** Call process_data(), isolating any exception it throws so one misbehaving slave can't
   *  abort the caller's per-cycle loop over the whole bus. Any EcMasterBase implementation
   *  should call this instead of process_data() directly. Returns false (and records the fault
   *  for collectDiagnostics()) if it threw. */
  inline
  bool processDataSafe(int index, uint8_t * domain_address)
  {
    try {
      process_data(index, domain_address);
      return true;
    } catch (const std::exception & e) {
      reportProcessDataFault(e.what());
      return false;
    } catch (...) {
      reportProcessDataFault("unknown exception");
      return false;
    }
  }

  /** Same isolation as processDataSafe(), for updateState(). */
  inline
  bool updateStateSafe()
  {
    try {
      updateState();
      return true;
    } catch (const std::exception & e) {
      reportProcessDataFault(e.what());
      return false;
    } catch (...) {
      reportProcessDataFault("unknown exception");
      return false;
    }
  }

  /** Clear a previously recorded process_data fault once a cycle completes cleanly. Callers
   *  loop process_data()/updateState() through processDataSafe()/updateStateSafe() and call
   *  this only once none of those calls faulted this cycle. */
  inline
  void clearProcessDataFault()
  {
    process_data_fault_.clear();
  }

  /** Fill in this slave's diagnostic status: generic EtherCAT-layer health (online, al_state)
   *  and a level derived from it. name/hardware_id are the caller's responsibility (it knows
   *  the component name; this slave doesn't). Subclasses that have more specific diagnostics
   *  (e.g. a CiA402 device state) should override, call the base implementation first to get
   *  the generic fields, then append/refine — not replace it outright. */
  virtual void collectDiagnostics(diagnostic_msgs::msg::DiagnosticStatus & status) const
  {
    status.level = (online_ && al_state_ == kAlStateOp) ?
      diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = online_ ? "online" : "offline";
    diagnostic_msgs::msg::KeyValue online_kv;
    online_kv.key = "online";
    online_kv.value = online_ ? "true" : "false";
    status.values.push_back(online_kv);
    diagnostic_msgs::msg::KeyValue al_state_kv;
    al_state_kv.key = "al_state";
    al_state_kv.value = alStateStr(al_state_);
    status.values.push_back(al_state_kv);
    if (!process_data_fault_.empty()) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "process_data fault: " + process_data_fault_;
      diagnostic_msgs::msg::KeyValue fault_kv;
      fault_kv.key = "process_data_fault";
      fault_kv.value = process_data_fault_;
      status.values.push_back(fault_kv);
    }
  }

  virtual bool setup_slave(
    std::unordered_map<std::string, std::string> slave_paramters,
    std::vector<double> * state_interface,
    std::vector<double> * command_interface)
  {
    state_interface_ptr_ = state_interface;
    command_interface_ptr_ = command_interface;
    parameters_ = slave_paramters;
    is_initialized_ = true;
    return true;
  }

  uint32_t get_vendor_id() {return vendor_id_;}
  uint32_t get_product_id() {return product_id_;}
  uint16_t get_alias() {return alias_;}
  uint16_t get_position() {return position_;}

  std::vector<SMConfig> get_sm_config()
  {
    return sm_config_;
  }

  std::vector<SdoConfigEntry> get_sdo_config()
  {
    return sdo_config_;
  }

  std::vector<SdoCheckEntry> get_sdo_check_config()
  {
    return sdo_check_config_;
  }

  std::vector<ethercat_interface::EcPdoChannelManager *> get_pdo_channels_info()
  {
    return pdo_channels_info_;
  }

  std::vector<pdo_info_t> get_pdo_info()
  {
    return pdo_info_;
  }

protected:
  std::vector<double> * state_interface_ptr_;
  std::vector<double> * command_interface_ptr_;
  std::unordered_map<std::string, std::string> parameters_;
  bool is_initialized_ = true;
  uint16_t alias_;        // < Slave alias.
  uint16_t position_;     // < Index after alias. If alias is zero, stores the ring position.
  uint32_t vendor_id_;   // < Slave vendor ID.
  uint32_t product_id_;  // < Slave product code.

  bool is_operational_ = false;
  bool is_alias_and_position_set_ = false;

  bool online_ = false;
  uint8_t al_state_ = 0;
  std::string process_data_fault_;

  /** Logs and records a process_data()/updateState() exception. Shared by processDataSafe()/
   *  updateStateSafe() so every EcMasterBase implementation gets the same log line and
   *  collectDiagnostics() reporting without re-implementing it. */
  inline
  void reportProcessDataFault(const std::string & what)
  {
    RCLCPP_ERROR(
        rclcpp::get_logger("EthercatDriver"),
        "Slave (alias %u, position %u) threw: %s", alias_, position_, what.c_str());
    process_data_fault_ = what;
  }


  std::vector<SdoConfigEntry> sdo_config_;
  std::vector<SdoCheckEntry> sdo_check_config_;
  std::vector<SMConfig> sm_config_;
  std::vector<ethercat_interface::EcPdoChannelManager *> pdo_channels_info_;
  std::vector<pdo_info_t> pdo_info_;
};

}  // namespace ethercat_interface
#endif  // ETHERCAT_INTERFACE__EC_SLAVE_BASE_HPP_
