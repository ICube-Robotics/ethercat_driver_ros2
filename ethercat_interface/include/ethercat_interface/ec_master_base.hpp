// Copyright 2023 ICUBE Laboratory, University of Strasbourg
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
// Author: Maciej Bednarczyk (mcbed.robotics@gmail.com)

#ifndef ETHERCAT_INTERFACE__EC_MASTER_BASE_HPP_
#define ETHERCAT_INTERFACE__EC_MASTER_BASE_HPP_

#include <cstdint>
#include <cstddef>
#include <string>
#include <memory>
#include <vector>
#include "ethercat_interface/ec_slave_base.hpp"
#include "ethercat_interface/ec_transfer.hpp"
#include "rclcpp/rclcpp.hpp"


namespace ethercat_interface
{

/** Per-slave AL state / link presence, as last observed by the master's periodic state check
 *  (throttled — see each master plugin's own check frequency). One entry per add_slave() call,
 *  in add order. Master-backend-agnostic: no ecrt.h/IgH types, so this survives swapping the
 *  master plugin (e.g. for a SOEM-backed implementation). */
struct EcSlaveStateInfo
{
  uint16_t alias = 0;
  uint16_t position = 0;
  /** AL state bitmask: INIT=1, PREOP=2, SAFEOP=4, OP=8 (the ETG.1000 standard EtherCAT AL
   *  state encoding; IgH's ec_al_state_t uses the same values). */
  uint8_t al_state = 0;
  /** false if the slave has stopped responding on the wire (e.g. cabling/power loss). */
  bool online = false;
  /** true once the slave has reached OP and started exchanging valid process data. */
  bool operational = false;
};

/** Last-observed master-wide link/AL state. Master-backend-agnostic (no ecrt.h types). */
struct EcMasterStateInfo
{
  bool link_up = false;
  uint32_t slaves_responding = 0;
  /** Aggregate AL-states bitmask, OR'd across all slaves (IgH ec_master_state_t.al_states
   *  encoding: INIT=1, PREOP=2, SAFEOP=4, OP=8). */
  uint8_t al_states = 0;
};

/** Last-observed process-data domain state (working counter / completeness).
 *  Master-backend-agnostic (no ecrt.h types). */
struct EcDomainStateInfo
{
  uint32_t working_counter = 0;
  /** 0 = ZERO (no data exchanged), 1 = INCOMPLETE, 2 = COMPLETE (matches IgH's
   *  ec_wc_state_t ordinal values: EC_WC_ZERO, EC_WC_INCOMPLETE, EC_WC_COMPLETE). */
  uint8_t wc_state = 0;
};

class EcMemoryEntry
{
public:
  std::string module_name;   //< Module.
  uint16_t alias;            //< Slave alias.
  uint16_t position;         //< Slave position.
  uint16_t index;            //< Channel index.
  uint16_t subindex;         //< Channel subindex.

public:
  inline
  std::string to_simple_string() const
  {
    return "( name= " + module_name + ", index= " +
           std::to_string(index) + ", subindex= " + std::to_string(subindex) + " )";
  }
};

class EcTransferEntry
{
public:
  EcMemoryEntry input;
  EcMemoryEntry output;
  size_t size;   //< Size of the exchange data.

public:
  inline
  std::string to_simple_string() const
  {
    return input.to_simple_string() + " -> " + output.to_simple_string() + " /  ( size= " +
           std::to_string(size) + " )";
  }
};

class EcTransferNet
{
public:
  std::string name;                    //< transfer net name (e.g. a safety net)
  std::string master;                  //< transfer master (e.g. the safety master of the net)
  std::vector<EcTransferEntry> transfers;   //< Data transfers (e.g. safety data transfers)

public:
  void reset(const std::string & new_name)
  {
    name = new_name;
    master = "";
    transfers.clear();
  }
};


class EcMasterBase
{
public:
  EcMasterBase() {}
  virtual ~EcMasterBase() {}

  /** \brief whether the underlying EtherCAT master was successfully obtained (init()
   *  succeeded and stop() has not since released it). Callers should check this before
   *  using the master. */
  virtual bool is_valid() const = 0;

  /** \brief Validate a slave — vendor/product identity against what's physically on the bus,
   *  plus its sdo_check: preconditions — WITHOUT registering it (no ecrt_master_slave_config,
   *  no PDO/domain registration, no side effects). Safe to call for every configured module
   *  before deciding which ones to add_slave(): both checks are addressed by ring position at
   *  the master level and need no prior slave configuration.
   *  \return true if the slave may be added; false if it failed a check (see logged detail). */
  virtual bool check_slave(std::shared_ptr<EcSlaveBase> slave) = 0;

  /** \brief add a slave device to the master */
  virtual bool add_slave(std::shared_ptr<EcSlaveBase> slave) = 0;

  /** \brief configure slave using SDO */
  virtual bool configure_slaves() = 0;

  virtual bool init(std::string iface) = 0;

  virtual bool start() = 0;

  virtual bool stop() = 0;

  /** @brief Deactivate the master (releasing the domain/slave-config objects it created for
   *  add_slave()/registerTransferInDomain()) so the bus drops out of cyclic exchange without
   *  releasing the master reservation itself. Per the EtherCAT spec, loss of cyclic process
   *  data should make each slave's Sync Manager Watchdog autonomously drop OP -> Safe-OP
   *  (still allowing mailbox/SDO access), rather than leaving slaves stuck at OP indefinitely
   *  as merely stopping the local cyclic loop (stop()) would.
   *  A subsequent add_slave()/start() cycle must rebuild the registration this frees; config
   *  SDOs already sent via configure_slaves() must NOT be resent.
   *  Must not be called in realtime context (blocking).
   *  \return true on success. */
  virtual bool deactivate() = 0;

  virtual bool reset() = 0;

  virtual bool spin_slaves_until_operational() = 0;

  /** \brief Blocking CoE SDO upload (read). ONLY valid during the "configure" phase — after
   *  configure_slaves() and before start() (or after stop(), before the next start()). Must
   *  NEVER be called while the master is activated (cyclic process-data exchange running):
   *  it blocks the calling thread on a mailbox round-trip, which would stall the real-time
   *  cycle. Implementations must refuse (return a negative value and log an error) rather
   *  than perform the transfer if called while activated.
   *  \return 0 on success (see ecrt_master_sdo_upload's return convention); negative if
   *          refused or the transfer failed. */
  virtual int upload_slave_sdo(
    uint16_t slave_position, uint16_t index, uint8_t sub_index,
    uint8_t * target, size_t target_size, size_t * result_size, uint32_t * abort_code) = 0;

  /** @brief Last-observed master state (link up/down, responding-slave count, aggregate AL
   *  states bitmask), as last updated by the periodic state check during
   *  read_process_data()/update(). No new bus transaction: returns the cached result of that
   *  periodic check. */
  virtual EcMasterStateInfo get_master_state() const = 0;

  /** @brief Last-observed domain state (working counter / completeness), as last updated by
   *  the periodic state check. No new bus transaction: returns the cached result. */
  virtual EcDomainStateInfo get_domain_state(uint32_t domain = 0) const = 0;

  /** @brief Last-observed per-slave AL state / online / operational, as last updated by the
   *  periodic state check. No new bus transaction: returns the cached result. */
  virtual std::vector<EcSlaveStateInfo> get_slave_states() const = 0;

  /** @brief Fill in the EcTransferInfo structures
  *
  * @param transfer_nets transfer nets
  *
  * \pre DomainInfo and domain_regs vectors must have been initialized and
  * activated. A call to EcMaster::activate() is required before calling
  * this function, to fill in the domain_regs vector offsets. Specifically
  * with IgH EtherCAT Master, the offset must have been initialized with the
  * ecrt_domain_reg_pdo_entry_list function.
  *
  * @throw std::runtime_error if some domain_info or some pdo_entry_reg are
  *  not valid
  */
  virtual void registerTransferInDomain(const std::vector<EcTransferNet> & transfer_nets) = 0;


  virtual bool read_process_data() = 0;

  virtual bool write_process_data() = 0;

  void set_ctrl_frequency(double frequency)
  {
    interval_ = 1000000000.0 / frequency;
  }

protected:
  uint32_t interval_;
};
}  // namespace ethercat_interface
#endif  // ETHERCAT_INTERFACE__EC_MASTER_BASE_HPP_
