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

#include "ethercat_master/ec_master_etherlab.hpp"
#include "ethercat_interface/ec_master.hpp"
#include "rclcpp/rclcpp.hpp"

#define EC_NEWTIMEVAL2NANO(TV) \
  (((TV).tv_sec - 946684800ULL) * 1000000000ULL + (TV).tv_nsec)

namespace ethercat_master
{

EtherlabMaster::DomainInfo::DomainInfo(ec_master_t * master)
{
  domain = ecrt_master_create_domain(master);
  if (domain == NULL) {
    printWarning("Failed to create domain");
    return;
  }

  const ec_pdo_entry_reg_t empty = {0};
  domain_regs.push_back(empty);
}


EtherlabMaster::DomainInfo::~DomainInfo()
{
  for (Entry & entry : entries) {
    delete[] entry.offset;
    delete[] entry.bit_position;
  }
}


EtherlabMaster::EtherlabMaster()
{
  interval_ = 0;
}

EtherlabMaster::~EtherlabMaster()
{
  for (auto & domain : domain_info_) {
    delete domain.second;
  }
}

bool EtherlabMaster::init(std::string master_interface)
{
  master_ = ecrt_request_master(std::stoi(master_interface));
  if (master_ == NULL) {
    printWarning("Failed to obtain master.");
    return false;
  }
  return true;
}

bool EtherlabMaster::add_slave(std::shared_ptr<ethercat_interface::EcSlave> slave)
{
  // configure slave in master

  slave_info_.emplace_back();

  slave_info_.back().slave = std::make_shared<EtherlabSlave>(slave);
  slave_info_.back().config = ecrt_master_slave_config(
    master_,
    slave_info_.back().slave->get_bus_alias(),
    slave_info_.back().slave->get_bus_position(),
    slave_info_.back().slave->get_vendor_id(),
    slave_info_.back().slave->get_product_id());
  if (slave_info_.back().config == NULL) {
    printWarning("Add slave. Failed to get slave configuration.");
    return false;
  }

  // check and setup dc

  if (slave_info_.back().slave->dc_sync()) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    ecrt_master_application_time(master_, EC_NEWTIMEVAL2NANO(t));
    ecrt_slave_config_dc(
      slave_info_.back().config,
      slave_info_.back().slave->dc_sync(),
      interval_,
      interval_ - (t.tv_nsec % (interval_)),
      0,
      0);
  }

  // check if slave has pdos
  size_t num_syncs = slave_info_.back().slave->sync_size();
  const ec_sync_info_t * syncs = slave_info_.back().slave->syncs();
  if (num_syncs > 0) {
    // configure pdos in slave
    int pdos_status = ecrt_slave_config_pdos(slave_info_.back().config, num_syncs, syncs);
    if (pdos_status) {
      printWarning("Add slave. Failed to configure PDOs");
      return false;
    }
  } else {
    printWarning(
      "Add slave. Sync size is zero for " +
      std::to_string(slave_info_.back().slave->get_bus_alias()) + ":" +
      std::to_string(slave_info_.back().slave->get_bus_position()));
  }

  // check if slave registered any pdos for the domain
  EtherlabSlave::DomainMap domain_map;
  slave_info_.back().slave->domains(domain_map);
  for (auto & iter : domain_map) {
    // get the domain info, create if necessary
    uint32_t domain_index = iter.first;
    DomainInfo * domain_info = domain_info_[domain_index];
    if (domain_info == NULL) {
      domain_info = new DomainInfo(master_);
      domain_info_[domain_index] = domain_info;
    }

    registerPDOInDomain(
      slave_info_.back().slave->get_bus_alias(), slave_info_.back().slave->get_bus_position(),
      iter.second, domain_info,
      slave_info_.back().slave);
  }

  return true;
}

bool EtherlabMaster::configure_slaves()
{
  for (auto i = 0ul; i < slave_info_.size(); i++) {
    for (auto & sdo : slave_info_[i].slave->get_sdo_config()) {
      uint8_t buffer[8];
      sdo.buffer_write(buffer);
      uint32_t abort_code;
      int ret = ecrt_master_sdo_download(
        master_,
        slave_info_[i].slave->get_bus_position(),
        sdo.index,
        sdo.sub_index,
        buffer,
        sdo.data_size(),
        &abort_code
      );

      if (ret) {
        RCLCPP_FATAL(
          rclcpp::get_logger("EtherlabMaster"),
          "Failed to download config SDO for module at position %i with Error: %d",
          slave_info_[i].slave->get_bus_position(),
          abort_code
        );
        return false;
      }
    }
  }

  return true;
}

void EtherlabMaster::registerPDOInDomain(
  uint16_t alias, uint16_t position,
  std::vector<uint32_t> & channel_indices,
  DomainInfo * domain_info,
  std::shared_ptr<EtherlabSlave> slave)
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

  EtherlabSlave::DomainMap domain_map;
  slave->domains(domain_map);

  // add to array of pdos registrations
  const ec_pdo_entry_info_t * pdo_regs = slave->channels();
  for (size_t i = 0; i < num_pdo_regs; ++i) {
    // create pdo entry in the domain
    ec_pdo_entry_reg_t & pdo_reg = domain_info->domain_regs[start_index + i];
    pdo_reg.alias = alias;
    pdo_reg.position = position;
    pdo_reg.vendor_id = slave->get_vendor_id();
    pdo_reg.product_code = slave->get_product_id();
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
  ec_pdo_entry_reg_t empty = {0};
  domain_info->domain_regs.back() = empty;
}

bool EtherlabMaster::start()
{
  // register domain
  for (auto & iter : domain_info_) {
    DomainInfo * domain_info = iter.second;
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
    domain_info->domain_pd = ecrt_domain_data(domain_info->domain);
    if (domain_info->domain_pd == NULL) {
      printWarning("Activate. Failed to retrieve domain process data.");
      return false;
    }
  }
  return true;
}

bool EtherlabMaster::stop()
{
  return true;
}

bool EtherlabMaster::spin_slaves_until_operational()
{
  // start after one second
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  t.tv_sec++;

  bool running = true;
  while (running) {
    // wait until next shot
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
    // update EtherCAT bus

    update();
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "updated!");

    // check if operational
    bool isAllInit = true;
    for (auto & slave_info : slave_info_) {
      isAllInit = isAllInit && slave_info.slave->initialized();
    }
    if (isAllInit) {
      running = false;
    }
    // calculate next shot. carry over nanoseconds into microseconds.
    t.tv_nsec += get_interval();
    while (t.tv_nsec >= 1000000000) {
      t.tv_nsec -= 1000000000;
      t.tv_sec++;
    }
  }

  return true;
}

void EtherlabMaster::update(uint32_t domain)
{
  // receive process data
  ecrt_master_receive(master_);

  DomainInfo * domain_info = domain_info_[domain];

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
    int channel_counter = 0;
    for (auto mi = 0ul; mi < (entry.slave)->get_pdo_config().size(); mi++) {
      for (auto ci = 0ul; ci < (entry.slave)->get_pdo_config()[mi].pdo_channel_config.size();
        ci++)
      {
        if ((entry.slave)->get_pdo_config()[mi].pdo_channel_config[ci].index != 0x0000) {
          (entry.slave)->process_data(
            mi, ci,
            domain_info->domain_pd + entry.offset[channel_counter]);
          channel_counter++;
        }
      }
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

bool EtherlabMaster::read_process_data()
{
  // receive process data
  ecrt_master_receive(master_);

  DomainInfo * domain_info = domain_info_[0];

  ecrt_domain_process(domain_info->domain);

  // check process data state (optional)
  checkDomainState(0);

  // check for master and slave state change
  if (update_counter_ % check_state_frequency_ == 0) {
    checkMasterState();
    checkSlaveStates();
  }

  // read and write process data
  for (DomainInfo::Entry & entry : domain_info->entries) {
    int channel_counter = 0;
    for (auto mi = 0ul; mi < (entry.slave)->get_pdo_config().size(); mi++) {
      for (auto ci = 0ul; ci < (entry.slave)->get_pdo_config()[mi].pdo_channel_config.size();
        ci++)
      {
        if ((entry.slave)->get_pdo_config()[mi].pdo_channel_config[ci].index != 0x0000) {
          (entry.slave)->process_data(
            mi, ci,
            domain_info->domain_pd + entry.offset[channel_counter]);
          channel_counter++;
        }
      }
    }
  }

  ++update_counter_;
  return true;
}

bool EtherlabMaster::write_process_data()
{
  DomainInfo * domain_info = domain_info_[0];
  // read and write process data
  for (DomainInfo::Entry & entry : domain_info->entries) {
    int channel_counter = 0;
    for (auto mi = 0ul; mi < (entry.slave)->get_pdo_config().size(); mi++) {
      for (auto ci = 0ul; ci < (entry.slave)->get_pdo_config()[mi].pdo_channel_config.size();
        ci++)
      {
        if ((entry.slave)->get_pdo_config()[mi].pdo_channel_config[ci].index != 0x0000) {
          (entry.slave)->process_data(
            mi, ci,
            domain_info->domain_pd + entry.offset[channel_counter]);
          channel_counter++;
        }
      }
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

  return true;
}

void EtherlabMaster::checkDomainState(uint32_t domain)
{
  DomainInfo * domain_info = domain_info_[domain];

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


void EtherlabMaster::checkMasterState()
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


void EtherlabMaster::checkSlaveStates()
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


void EtherlabMaster::printWarning(const std::string & message)
{
  std::cout << "WARNING. Master. " << message << std::endl;
}

}  // namespace ethercat_master

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ethercat_master::EtherlabMaster, ethercat_interface::EcMaster)
