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

  bool add_slave(ethercat_interface::EcSlave * slave);

  bool configure_slaves();

  bool start();

  void update(uint32_t domain = 0);

  bool spin_slaves_until_operational();

  /** run a control loop of update() and user_callback(), blocking.
   *  call activate and setThreadHighPriority/RealTime first. */
  typedef void (* SIMPLECAT_CONTRL_CALLBACK)(void);
  void run(SIMPLECAT_CONTRL_CALLBACK user_callback);

  /** stop the control loop. use within callback, or from a separate thread. */
  bool stop();

  /** time of last ethercat update, since calling run. stops if stop called.
   *  returns actual time. use elapsedCycles()/frequency for discrete time at last update. */
  virtual double elapsedTime();

  /** number of EtherCAT updates since calling run. */
  virtual uint64_t elapsedCycles();

  /** add ctr-c exit callback.
    * default exits the run loop and prints timing */
  typedef void (* SIMPLECAT_EXIT_CALLBACK)(int);
  static void setCtrlCHandler(SIMPLECAT_EXIT_CALLBACK user_callback = NULL);

  /** set the thread to a priority of -19
   *  priority range is -20 (highest) to 19 (lowest) */
  static void setThreadHighPriority();

  /** set the thread to real time (FIFO)
   *  thread cannot be preempted.
   *  set priority as 49 (kernel and interrupts are 50) */
  static void setThreadRealTime();

  void set_ctrl_frequency(double frequency)
  {
    interval_ = 1000000000.0 / frequency;
  }

  uint32_t get_interval() {return interval_;}

  bool read_process_data();
  bool write_process_data();

private:
  /** true if running */
  volatile bool running_ = false;

  /** start and current time */
  std::chrono::time_point<std::chrono::system_clock> start_t_, curr_t_;

  // EtherCAT Control

  /** register a domain of the slave */
  struct DomainInfo;
  void registerPDOInDomain(
    uint16_t alias, uint16_t position,
    std::vector<uint32_t> & channel_indices,
    DomainInfo * domain_info,
    EtherlabSlave * slave);

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
      EtherlabSlave * slave = NULL;
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
    EtherlabSlave * slave = NULL;
    ec_slave_config_t * config = NULL;
    ec_slave_config_state_t config_state = {0};
  };

  std::vector<SlaveInfo> slave_info_;

  /** counter of control loops */
  uint64_t update_counter_ = 0;

  /** frequency to check for master or slave state change.
   *  state checked every frequency_ control loops */
  uint32_t check_state_frequency_ = 10;

  uint32_t interval_;

  std::vector<std::shared_ptr<EtherlabSlave>> slave_list_;
};

}  // namespace ethercat_master

#endif  // ETHERCAT_MASTER__EC_MASTER_ETHERLAB_HPP_
