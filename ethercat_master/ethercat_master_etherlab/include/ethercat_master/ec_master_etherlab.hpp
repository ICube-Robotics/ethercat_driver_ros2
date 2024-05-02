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

#include "ethercat_interface/ec_master.hpp"
#include "ethercat_interface/ec_slave.hpp"
#include "ethercat_master/ec_slave_etherlab.hpp"

namespace ethercat_master
{

class EtherlabMaster : public ethercat_interface::EcMaster
{
public:
  EtherlabMaster();
  ~EtherlabMaster();

  bool init(std::string master_interface = "0");

  bool add_slave(std::shared_ptr<ethercat_interface::EcSlave> slave);

  bool configure_slaves();

  bool start();

  void update(uint32_t domain = 0);

  bool spin_slaves_until_operational();

  /** stop the control loop.
   */
  bool stop();

  uint32_t get_interval() {return interval_;}

  bool read_process_data();
  bool write_process_data();

private:
  // EtherCAT Control

  /** register a domain of the slave */
  struct DomainInfo;
  void registerPDOInDomain(
    uint16_t alias, uint16_t position,
    std::vector<uint32_t> & channel_indices,
    DomainInfo * domain_info,
    std::shared_ptr<EtherlabSlave> slave);

  /** check for change in the domain state */
  void checkDomainState(uint32_t domain);

  /** check for change in the master state */
  void checkMasterState();

  /** check for change in the slave states */
  void checkSlaveStates();

  /** print warning message to terminal */
  static void printWarning(const std::string & message);

  /** EtherCAT master data */
  ec_master_t * master_ = NULL;
  ec_master_state_t master_state_ = {};

  /** data for a single domain */
  struct DomainInfo
  {
    explicit DomainInfo(ec_master_t * master);
    ~DomainInfo();

    ec_domain_t * domain = NULL;
    ec_domain_state_t domain_state = {};
    uint8_t * domain_pd = NULL;

    /** domain pdo registration array.
     *  do not modify after active(), or may invalidate */
    std::vector<ec_pdo_entry_reg_t> domain_regs;

    /** slave's pdo entries in the domain */
    struct Entry
    {
      std::shared_ptr<EtherlabSlave> slave = nullptr;
      int num_pdos = 0;
      uint32_t * offset = NULL;
      uint32_t * bit_position = NULL;
    };

    std::vector<Entry> entries;
  };

  /** map from domain index to domain info */
  std::map<uint32_t, DomainInfo *> domain_info_;

  /** data needed to check slave state */
  struct SlaveInfo
  {
    std::shared_ptr<EtherlabSlave> slave = nullptr;
    ec_slave_config_t * config = NULL;
    ec_slave_config_state_t config_state = {0};
  };

  std::vector<SlaveInfo> slave_info_;

  /** counter of control loops */
  uint64_t update_counter_ = 0;

  /** frequency to check for master or slave state change.
   *  state checked every frequency_ control loops */
  uint32_t check_state_frequency_ = 10;
};

}  // namespace ethercat_master

#endif  // ETHERCAT_MASTER__EC_MASTER_ETHERLAB_HPP_
