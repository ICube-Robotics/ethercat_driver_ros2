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

#include "ethercat_interface/ec_master.hpp"
#include "ethercat_interface/ec_slave.hpp"

#include <unistd.h>
#include <sys/resource.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <time.h>
#include <sys/mman.h>
#include <string.h>
#include <iostream>
#include <sstream>

#define EC_NEWTIMEVAL2NANO(TV) \
  (((TV).tv_sec - 946684800ULL) * 1000000000ULL + (TV).tv_nsec)

namespace ethercat_interface
{

EcMaster::DomainInfo::DomainInfo(ec_master_t * master)
{
  domain = ecrt_master_create_domain(master);
  if (domain == NULL) {
    printWarning("Failed to create domain");
    return;
  }

  const ec_pdo_entry_reg_t empty = {0, 0, 0, 0, 0, 0, nullptr, nullptr};
  domain_regs.push_back(empty);
}


EcMaster::DomainInfo::~DomainInfo()
{
  for (Entry & entry : entries) {
    delete[] entry.offset;
    delete[] entry.bit_position;
  }
}


EcMaster::EcMaster(const int master)
{
  master_ = ecrt_request_master(master);
  if (master_ == NULL) {
    printWarning("Failed to obtain master.");
    return;
  }
  interval_ = 0;
}

EcMaster::~EcMaster()
{
  /*
  for (SlaveInfo & slave : slave_info_) {
    //TODO verify what this piece of code was here for
  }
  */
  for (auto & domain : domain_info_) {
    if (domain.second != NULL) {
      delete domain.second;
    }
  }
}

void EcMaster::addSlave(uint16_t alias, uint16_t position, EcSlave * slave)
{
  // configure slave in master

  SlaveInfo slave_info;
  slave_info.slave = slave;
  slave_info.config = ecrt_master_slave_config(
    master_, alias, position,
    slave->vendor_id_,
    slave->product_id_);
  if (slave_info.config == NULL) {
    printWarning("Add slave. Failed to get slave configuration.");
    return;
  }

  // check and setup dc

  if (slave->assign_activate_dc_sync()) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    ecrt_master_application_time(master_, EC_NEWTIMEVAL2NANO(t));
    ecrt_slave_config_dc(
      slave_info.config,
      slave->assign_activate_dc_sync(),
      interval_,
      interval_ - (t.tv_nsec % (interval_)),
      0,
      0);
  }

  slave_info_.push_back(slave_info);

  // check if slave has pdos
  size_t num_syncs = slave->syncSize();
  const ec_sync_info_t * syncs = slave->syncs();
  if (num_syncs > 0) {
    // configure pdos in slave
    int pdos_status = ecrt_slave_config_pdos(slave_info.config, num_syncs, syncs);
    if (pdos_status) {
      printWarning("Add slave. Failed to configure PDOs");
      return;
    }
  } else {
    printWarning(
      "Add slave. Sync size is zero for " +
      std::to_string(alias) + ":" + std::to_string(position));
  }

  // check if slave registered any pdos for the domain
  EcSlave::DomainMap domain_map;
  slave->domains(domain_map);
  for (auto & iter : domain_map) {
    // get the domain info, create if necessary
    uint32_t domain_index = iter.first;
    DomainInfo * domain_info = NULL;
    if (domain_info_.count(domain_index)) {
      domain_info = domain_info_.at(domain_index);
    }
    if (domain_info == NULL) {
      domain_info = new DomainInfo(master_);
      domain_info_[domain_index] = domain_info;
    }

    registerPDOInDomain(
      alias, position,
      iter.second, domain_info,
      slave);
  }
}

int EcMaster::configSlaveSdo(
  uint16_t slave_position, SdoConfigEntry sdo_config,
  uint32_t * abort_code)
{
  uint8_t buffer[8];
  sdo_config.buffer_write(buffer);
  int ret = ecrt_master_sdo_download(
    master_,
    slave_position,
    sdo_config.index,
    sdo_config.sub_index,
    buffer,
    sdo_config.data_size(),
    abort_code
  );
  return ret;
}

void EcMaster::registerPDOInDomain(
  uint16_t alias, uint16_t position,
  std::vector<uint32_t> & channel_indices,
  DomainInfo * domain_info,
  EcSlave * slave)
{
  // expand the size of the domain
  uint32_t num_pdo_regs = channel_indices.size();
  size_t start_index = domain_info->domain_regs.size() - 1;  // empty element at end
  domain_info->domain_regs.resize(domain_info->domain_regs.size() + num_pdo_regs);

  // create a new entry in the domain
  DomainInfo::Entry domain_entry;
  domain_entry.slave = slave;
  domain_entry.num_pdos = num_pdo_regs;
  domain_entry.offset = new uint32_t[num_pdo_regs];
  domain_entry.bit_position = new uint32_t[num_pdo_regs];
  domain_info->entries.push_back(domain_entry);

  EcSlave::DomainMap domain_map;
  slave->domains(domain_map);

  // add to array of pdos registrations
  const ec_pdo_entry_info_t * pdo_regs = slave->channels();
  for (size_t i = 0; i < num_pdo_regs; ++i) {
    // create pdo entry in the domain
    ec_pdo_entry_reg_t & pdo_reg = domain_info->domain_regs[start_index + i];
    pdo_reg.alias = alias;
    pdo_reg.position = position;
    pdo_reg.vendor_id = slave->vendor_id_;
    pdo_reg.product_code = slave->product_id_;
    pdo_reg.index = pdo_regs[channel_indices[i]].index;
    pdo_reg.subindex = pdo_regs[channel_indices[i]].subindex;
    pdo_reg.offset = &(domain_entry.offset[i]);
    pdo_reg.bit_position = &(domain_entry.bit_position[i]);


    // print the domain pdo entry
    std::cout << "{" << pdo_reg.alias << ", " << pdo_reg.position;
    std::cout << ", 0x" << std::hex << pdo_reg.vendor_id;
    std::cout << ", 0x" << std::hex << pdo_reg.product_code;
    std::cout << ", 0x" << std::hex << pdo_reg.index;
    std::cout << ", 0x" << std::hex << static_cast<int>(pdo_reg.subindex);
    std::cout << "}" << std::dec << std::endl;
  }

  // set the last element to null
  ec_pdo_entry_reg_t empty = {0, 0, 0, 0, 0, 0, nullptr, nullptr};
  domain_info->domain_regs.back() = empty;
}

bool EcMaster::activate()
{
  // register domain
  for (auto & iter : domain_info_) {
    DomainInfo * domain_info = iter.second;
    if (domain_info == NULL) {
      throw std::runtime_error("Null domain info: " + std::to_string(iter.first));
    }
    bool domain_status = ecrt_domain_reg_pdo_entry_list(
      domain_info->domain,
      &(domain_info->domain_regs[0]));
    if (domain_status) {
      printWarning("Activate. Failed to register domain PDO entries.");
      return false;
    }
  }
  // set application time
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  ecrt_master_application_time(master_, EC_NEWTIMEVAL2NANO(t));

  // activate master
  bool activate_status = ecrt_master_activate(master_);
  if (activate_status) {
    printWarning("Activate. Failed to activate master.");
    return false;
  }

  // retrieve domain data
  for (auto & iter : domain_info_) {
    DomainInfo * domain_info = iter.second;
    if (domain_info == NULL) {
      throw std::runtime_error("Null domain info: " + std::to_string(iter.first));
    }
    domain_info->domain_pd = ecrt_domain_data(domain_info->domain);
    if (domain_info->domain_pd == NULL) {
      printWarning("Activate. Failed to retrieve domain process data.");
      return false;
    }
  }
  return true;
}

void EcMaster::update(uint32_t domain)
{
  // receive process data
  ecrt_master_receive(master_);

  DomainInfo * domain_info = domain_info_.at(domain);
  if (domain_info == NULL) {
    throw std::runtime_error("Null domain info: " + std::to_string(domain));
  }

  ecrt_domain_process(domain_info->domain);

  // check process data state (optional)
  checkDomainState(domain);

  // check for master and slave state change
  if (update_counter_ % check_state_frequency_ == 0) {
    checkMasterState();
    checkSlaveStates();
  }

  // read and write process data
  for (DomainInfo::Entry & entry : domain_info->entries) {
    for (int i = 0; i < entry.num_pdos; ++i) {
      (entry.slave)->processData(i, domain_info->domain_pd + entry.offset[i]);
    }
  }

  struct timespec t;

  clock_gettime(CLOCK_REALTIME, &t);
  ecrt_master_application_time(master_, EC_NEWTIMEVAL2NANO(t));
  ecrt_master_sync_reference_clock(master_);
  ecrt_master_sync_slave_clocks(master_);

  // send process data
  ecrt_domain_queue(domain_info->domain);
  ecrt_master_send(master_);

  ++update_counter_;
}

void EcMaster::readData(uint32_t domain)
{
  // receive process data
  ecrt_master_receive(master_);

  DomainInfo * domain_info = domain_info_.at(domain);
  if (domain_info == NULL) {
    throw std::runtime_error("Null domain info: " + std::to_string(domain));
  }

  ecrt_domain_process(domain_info->domain);

  // check process data state (optional)
  checkDomainState(domain);

  // check for master and slave state change
  if (update_counter_ % check_state_frequency_ == 0) {
    checkMasterState();
    checkSlaveStates();
  }

  // read and write process data
  for (DomainInfo::Entry & entry : domain_info->entries) {
    for (int i = 0; i < entry.num_pdos; ++i) {
      (entry.slave)->processData(i, domain_info->domain_pd + entry.offset[i]);
    }
  }

  ++update_counter_;
}

void EcMaster::writeData(uint32_t domain)
{
  DomainInfo * domain_info = domain_info_.at(domain);
  if (domain_info == NULL) {
    throw std::runtime_error("Null domain info: " + std::to_string(domain));
  }

  // read and write process data
  for (DomainInfo::Entry & entry : domain_info->entries) {
    for (int i = 0; i < entry.num_pdos; ++i) {
      (entry.slave)->processData(i, domain_info->domain_pd + entry.offset[i]);
    }
  }

  struct timespec t;

  clock_gettime(CLOCK_REALTIME, &t);
  ecrt_master_application_time(master_, EC_NEWTIMEVAL2NANO(t));
  ecrt_master_sync_reference_clock(master_);
  ecrt_master_sync_slave_clocks(master_);

  // send process data
  ecrt_domain_queue(domain_info->domain);
  ecrt_master_send(master_);
}

void EcMaster::setCtrlCHandler(SIMPLECAT_EXIT_CALLBACK user_callback)
{
  // ctrl c handler
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = user_callback;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);
}

void EcMaster::run(SIMPLECAT_CONTRL_CALLBACK user_callback)
{
  // start after one second
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  t.tv_sec++;

  running_ = true;
  start_t_ = std::chrono::system_clock::now();
  while (running_) {
    // wait until next shot
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);

    // update EtherCAT bus
    this->update();

    // get actual time
    curr_t_ = std::chrono::system_clock::now();

    // user callback
    user_callback();

    // calculate next shot. carry over nanoseconds into microseconds.
    t.tv_nsec += interval_;
    while (t.tv_nsec >= 1000000000) {
      t.tv_nsec -= 1000000000;
      t.tv_sec++;
    }
  }
}

double EcMaster::elapsedTime()
{
  std::chrono::duration<double> elapsed_seconds = curr_t_ - start_t_;
  return elapsed_seconds.count() - 1.0;  // started after 1 second
}

uint64_t EcMaster::elapsedCycles()
{
  return update_counter_;
}

void EcMaster::setThreadHighPriority()
{
  pid_t pid = getpid();
  int priority_status = setpriority(PRIO_PROCESS, pid, -19);
  if (priority_status) {
    printWarning("setThreadHighPriority. Failed to set priority.");
    return;
  }
}

void EcMaster::setThreadRealTime()
{
  /* Declare ourself as a real time task, priority 49.
      PRREMPT_RT uses priority 50
      for kernel tasklets and interrupt handler by default */
  struct sched_param param;
  param.sched_priority = 49;
  // pthread_t this_thread = pthread_self();
  if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
    perror("sched_setscheduler failed");
    exit(-1);
  }

  /* Lock memory */
  if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
    perror("mlockall failed");
    exit(-2);
  }

  /* Pre-fault our stack
      8*1024 is the maximum stack size
      which is guaranteed safe to access without faulting */
  constexpr unsigned int MAX_SAFE_STACK = 8 * 1024;
  unsigned char dummy[MAX_SAFE_STACK];
  memset(dummy, 0, MAX_SAFE_STACK);
}

void EcMaster::checkDomainState(uint32_t domain)
{
  DomainInfo * domain_info = domain_info_.at(domain);
  if (domain_info == NULL) {
    throw std::runtime_error("Null domain info: " + std::to_string(domain));
  }

  ec_domain_state_t ds;
  ecrt_domain_state(domain_info->domain, &ds);

  if (ds.working_counter != domain_info->domain_state.working_counter) {
    printf("Domain: WC %u.\n", ds.working_counter);
  }
  if (ds.wc_state != domain_info->domain_state.wc_state) {
    printf("Domain: State %u.\n", ds.wc_state);
  }
  domain_info->domain_state = ds;
}


void EcMaster::checkMasterState()
{
  ec_master_state_t ms;
  ecrt_master_state(master_, &ms);

  if (ms.slaves_responding != master_state_.slaves_responding) {
    printf("%u slave(s).\n", ms.slaves_responding);
  }
  if (ms.al_states != master_state_.al_states) {
    printf("Master AL states: 0x%02X.\n", ms.al_states);
  }
  if (ms.link_up != master_state_.link_up) {
    printf("Link is %s.\n", ms.link_up ? "up" : "down");
  }
  master_state_ = ms;
}


void EcMaster::checkSlaveStates()
{
  for (SlaveInfo & slave : slave_info_) {
    ec_slave_config_state_t s;
    ecrt_slave_config_state(slave.config, &s);

    if (s.al_state != slave.config_state.al_state) {
      // this spams the terminal at initialization.
      printf("Slave: State 0x%02X.\n", s.al_state);
    }
    if (s.online != slave.config_state.online) {
      printf("Slave: %s.\n", s.online ? "online" : "offline");
    }
    if (s.operational != slave.config_state.operational) {
      printf("Slave: %soperational.\n", s.operational ? "" : "Not ");
      slave.slave->set_state_is_operational(s.operational ? true : false);
    }
    slave.config_state = s;
  }
}


void EcMaster::printWarning(const std::string & message)
{
  std::cout << "WARNING. Master. " << message << std::endl;
}

}  // namespace ethercat_interface
