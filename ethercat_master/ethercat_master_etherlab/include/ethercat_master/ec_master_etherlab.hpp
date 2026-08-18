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

#ifndef ETHERCAT_MASTER__EC_MASTER_ETHERLAB_HPP_
#define ETHERCAT_MASTER__EC_MASTER_ETHERLAB_HPP_

#include <ecrt.h>

#include <time.h>
#include <string>
#include <vector>
#include <map>
#include <chrono>
#include <memory>

#include "ethercat_interface/ec_master_base.hpp"
#include "ethercat_interface/ec_slave_base.hpp"
#include "ethercat_master/ec_slave_etherlab.hpp"

namespace ethercat_master
{

inline uint64_t EC_NEWTIMEVAL2NANO(struct timespec & TV)
{
  return (TV.tv_sec - 946684800ULL) * 1000000000ULL + TV.tv_nsec;
}

/** Data for a single domain */
struct DomainInfo
{
  explicit DomainInfo(ec_master_t * master);
  ~DomainInfo();

  ec_domain_t * domain = NULL;
  ec_domain_state_t domain_state = {};
  uint8_t * domain_pd = NULL;  //< pointer to process domain data

  /** domain pdo registration array.
   *  do not modify after active(), or may invalidate */
  std::vector<ec_pdo_entry_reg_t> domain_regs;

  /** slave's pdo and in memory data entries in the domain */
  struct Entry
  {
    std::shared_ptr<EtherlabSlave> slave = nullptr;
    int num_pdos = 0;
    uint32_t * offset = NULL;
    uint32_t * bit_position = NULL;
    int num_in_memory_data = 0;
    uint32_t * offset_in_memory = NULL;
  };

  std::vector<Entry> entries;
};

class EtherlabMaster : public ethercat_interface::EcMasterBase
{
public:
  EtherlabMaster();
  ~EtherlabMaster();

  bool is_valid() const {return master_ != NULL;}

  bool init(std::string master_interface = "0");

  bool check_slave(std::shared_ptr<ethercat_interface::EcSlaveBase> slave);

  bool add_slave(std::shared_ptr<ethercat_interface::EcSlaveBase> slave);


  bool configure_slaves();

  bool start();

  bool reset();

  /** stop the control loop.
   */
  bool stop();

  /** see EcMasterBase::deactivate() */
  bool deactivate();

  bool read_process_data();
  bool write_process_data();

  /** Blocking CoE SDO upload (read). Refuses (returns negative, logs an error) unless called
   *  during the configure phase — after configure_slaves() and before start(), or after
   *  stop(). See EcMasterBase::upload_slave_sdo(). */
  int upload_slave_sdo(
    uint16_t slave_position, uint16_t index, uint8_t sub_index,
    uint8_t * target, size_t target_size, size_t * result_size, uint32_t * abort_code);

  ethercat_interface::EcMasterStateInfo get_master_state() const;
  ethercat_interface::EcDomainStateInfo get_domain_state(uint32_t domain = 0) const;
  std::vector<ethercat_interface::EcSlaveStateInfo> get_slave_states() const;

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
  void registerTransferInDomain(
    const std::vector<ethercat_interface::EcTransferNet> & transfer_nets);

protected:
/** @brief Proceed to the transfer of all the data declared in transfers_.
   */
  void transferAll();
  /** @brief Output the memory content of the all the domains
   * (available for pedagogic and debug purposes)
   *
   * @param[out] os Output stream
  */
  // void printMemoryFrames(std::ostream & os);

  /** @brief Get pointer on memory frame for a certain point
   * in the frame defined by a slave position, an index and a subindex
   */
  /*uint8_t * getMemoryStart(
    const uint16_t position,
    const uint16_t index,
    const uint16_t subindex);*/

  /** @brief Output memory n bytes of memory from a certain point in
   * the frame defined by a slave position, an index, subindex */
  /*void printMemoryFrame(
    const uint16_t position,
    const uint16_t index,
    const uint16_t subindex,
    const size_t n,
    bool binary = false,
    std::ostream & os = std::cout);*/

protected:
  /** start and current time */
  // std::chrono::time_point<std::chrono::system_clock> start_t_, curr_t_;

private:
  // EtherCAT Control

  /** Vendor/product identity check against what's physically on the bus at this slave's ring
   *  position — shared by add_slave() and check_slave() so both see identical behavior. No
   *  side effects; safe before ecrt_master_slave_config(). */
  bool checkSlaveIdentity(std::shared_ptr<ethercat_interface::EcSlaveBase> slave) const;

  /** Run a slave's sdo_check: preconditions (read + allowed-value match). No side effects;
   *  addressed by ring position, safe before ecrt_master_slave_config(). */
  bool checkSlaveSdoChecks(std::shared_ptr<ethercat_interface::EcSlaveBase> slave) const;

  /** Resolve (alias, position) to an absolute ring position, as required by the blocking
   *  SDO calls (they address by absolute position only). alias 0 returns position
   *  unchanged. Negative if the alias is not found on the bus. No side effects. */
  int resolveAbsolutePosition(uint16_t alias, uint16_t position) const;

  /** register a domain of the slave */
  void registerPDOInDomain(
    std::vector<uint32_t> & channel_indices,
    DomainInfo *domain_info,
    std::shared_ptr<EtherlabSlave> slave);

  /** check for change in the domain state */
  void checkDomainState(uint32_t domain);

  /** check for change in the master state */
  void checkMasterState();

  /** check for change in the slave states */
  void checkSlaveStates();

  /** print warning message to terminal */
  inline
  static void printWarning(const std::string & message)
  {
    RCLCPP_WARN(rclcpp::get_logger("EthercatDriver"), "WARNING. Master. %s", message.c_str());
  }

  /** print error message to terminal */
  inline
  static void printError(const std::string & message)
  {
    RCLCPP_ERROR(rclcpp::get_logger("EthercatDriver"), "ERROR. Master. %s", message.c_str());
  }

  /** @brief Check the validity of the domain info and the ec_pdo_entry_reg_t
   * and throw an exception if not valid.
   *
   * @param domain_info Domain info
   * @param pdo_entry_reg PDO entry registration
   *
   * @throw std::runtime_error if domain_info or pdo_entry_reg is not valid
  */
  /*void checkDomainInfoValidity(
    const DomainInfo & domain_info,
    const ec_pdo_entry_reg_t & pdo_entry_reg);*/


  /** EtherCAT master data */
  ec_master_t * master_ = NULL;
  ec_master_state_t master_state_ = {};

  /** true from a successful start() until stop(). upload_slave_sdo() refuses to run a
   *  blocking mailbox transfer while this is true — see EcMasterBase::upload_slave_sdo(). */
  bool activated_ = false;

  /** map from domain index to domain info */
  std::map<uint32_t, DomainInfo *> domain_info_;

  /** data needed to check slave state */
  struct SlaveInfo
  {
    std::shared_ptr<EtherlabSlave> slave = nullptr;
    ec_slave_config_t * config = NULL;
    ec_slave_config_state_t config_state = {0, 0, 0};
  };


  std::vector<SlaveInfo> slave_info_;

  /** counter of control loops */
  uint64_t update_counter_ = 0;

  /** frequency to check for master or slave state change.
   *  state checked every frequency_ control loops */
  uint32_t check_state_frequency_ = 10;


  /** Data transfers (necessary for transfer communication) */
  std::vector<ethercat_interface::EcTransferInfo> transfers_;

protected:
  friend struct DomainInfo;
  friend struct ethercat_interface::EcTransferInfo;
};

}  // namespace ethercat_master

#endif  // ETHERCAT_MASTER__EC_MASTER_ETHERLAB_HPP_
