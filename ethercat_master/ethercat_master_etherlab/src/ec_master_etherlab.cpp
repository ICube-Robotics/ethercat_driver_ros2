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
#include "ethercat_interface/ec_master_base.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ethercat_master
{

DomainInfo::DomainInfo(ec_master_t *master)
{
  domain = ecrt_master_create_domain(master);
  if (domain == NULL) {
    EtherlabMaster::printWarning("Failed to create domain");
    return;
  }

  const ec_pdo_entry_reg_t empty = {0, 0, 0, 0, 0, 0, nullptr, nullptr};
  domain_regs.push_back(empty);
}

DomainInfo::~DomainInfo()
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
    if (domain.second != NULL) {
      delete domain.second;
    }
  }

  for (auto i = 0ul; i < slave_info_.size(); i++) {
    if (slave_info_[i].slave != nullptr) {
      slave_info_[i].slave.reset();
    }
  }

  // Release the EtherCAT master so the kernel module can be reused — without this the
  // master stays locked after the process exits, preventing re-initialization on restart
  // without unloading the kernel module (critical in multi-master setups).
  if (master_) {
    ecrt_release_master(master_);
  }
}

bool EtherlabMaster::init(std::string master_interface)
{
  master_ = ecrt_request_master(std::stoul(master_interface));
  if (master_ == NULL) {
    RCLCPP_FATAL(
          rclcpp::get_logger("EtherlabMaster"),
          "Failed to obtain master.");
    return false;
  }
  interval_ = 0;
  RCLCPP_INFO(
        rclcpp::get_logger("EtherlabMaster"),
        "master %lu ready.", std::stoul(master_interface));
  return true;
}

bool EtherlabMaster::add_slave(std::shared_ptr<ethercat_interface::EcSlaveBase> slave)
{
  if (false == slave->isAliasAndPositionSet()) {
    std::string error_message = "Alias and position not set for slave (vendor id=" +
      std::to_string(slave->get_vendor_id()) + ",product_code=" +
      std::to_string(slave->get_product_id()) + ").";
    throw std::runtime_error(error_message);
  }

  // Verify the drive actually on the bus matches what the slave_config declares (vendor +
  // product). The master would otherwise leave a mismatched slave unconfigured and it would
  // silently never reach OPERATIONAL; surface it loudly and refuse to configure the drive
  // instead. ecrt_master_get_slave() addresses by ABSOLUTE ring position, which equals
  // slave->get_position() only for alias 0 (for a non-zero alias, position is relative to
  // that alias), so the identity check is enforced for alias-0 slaves and skipped for
  // aliased ones.
  if (slave->get_alias() == 0) {
    ec_slave_info_t info{};
    if (ecrt_master_get_slave(master_, slave->get_position(), &info) != 0) {
      std::ostringstream msg;
      msg << "Add slave. No slave found at ring position " << slave->get_position() << std::hex
          << " (slave_config expects vendor=0x" << slave->get_vendor_id()
          << ", product=0x" << slave->get_product_id()
          << "). Refusing to configure this drive."
          << " Is the drive powered and connected on the bus?";
      printError(msg.str());
      return false;
    }
    if (info.vendor_id != slave->get_vendor_id() || info.product_code != slave->get_product_id()) {
      std::ostringstream msg;
      msg << "Add slave. Identity mismatch at ring position " << slave->get_position()
          << std::hex << ": the drive on the bus is vendor=0x" << info.vendor_id
          << ", product=0x" << info.product_code << " (revision 0x" << info.revision_number
          << ", \"" << info.name << "\"), but the slave_config expects vendor=0x"
          << slave->get_vendor_id() << ", product=0x" << slave->get_product_id()
          << ". Refusing to configure this drive — check the slave_config matches your hardware.";
      printError(msg.str());
      return false;
    }
  } else {
    std::ostringstream msg;
    msg << "Add slave at alias " << slave->get_alias() << " position " << slave->get_position()
        << ": skipping the vendor/product identity check (only enforced for alias-0 slaves "
      "addressed by absolute ring position).";
    printWarning(msg.str());
  }

  // configure slave in master
  SlaveInfo slave_info;

  // slave_info_.emplace_back();

  slave_info.slave = std::make_shared<EtherlabSlave>(slave);
  slave_info.config = ecrt_master_slave_config(
        master_,
        slave->get_alias(),
        slave->get_position(),
        slave->get_vendor_id(),
        slave->get_product_id());
  if (slave_info.config == NULL) {
    printError("Add slave. Failed to get slave configuration.");
    return false;
  }

    // check and setup dc

  if (slave_info.slave->assign_activate_dc_sync()) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
      // ecrt_master_application_time(master_, EC_NEWTIMEVAL2NANO(t));
    ecrt_slave_config_dc(
          slave_info.config,
          slave_info.slave->assign_activate_dc_sync(),
          interval_,
          interval_ / 2,
          0,
          0);
  }

  slave_info_.push_back(slave_info);

    // check if slave has pdos
  size_t num_syncs = slave_info.slave->sync_size();
  const ec_sync_info_t *syncs = slave_info.slave->syncs();
  if (num_syncs > 0) {
      // configure pdos in slave
    int pdos_status = ecrt_slave_config_pdos(slave_info.config, num_syncs, syncs);
    if (pdos_status) {
      printError("Add slave. Failed to configure PDOs");
      return false;
    }
  } else {
    printWarning(
          "Add slave. Sync size is zero for " +
          std::to_string(slave->get_alias()) + ":" +
          std::to_string(slave->get_position()));
  }

    // check if slave registered any pdos for the domain
  EtherlabSlave::DomainMap domain_map;
  slave_info.slave->domains(domain_map);
  for (auto & iter : domain_map) {
      // get the domain info, create if necessary
    uint32_t domain_index = iter.first;
    DomainInfo *domain = NULL;
    if (domain_info_.count(domain_index)) {
      domain = domain_info_.at(domain_index);
    }
    if (domain == NULL) {
      domain = new DomainInfo(master_);
      domain_info_[domain_index] = domain;
    }
    registerPDOInDomain(
          iter.second, domain,
          slave_info.slave);
  }

  return true;
}

bool EtherlabMaster::configure_slaves()
{
  for (auto i = 0ul; i < slave_info_.size(); i++) {
    for (auto & sdo : slave_info_[i].slave->get_slave()->get_sdo_config()) {
      uint8_t buffer[8];
      sdo.buffer_write(buffer);
      uint32_t abort_code;
      int ret = ecrt_master_sdo_download(
            master_,
            slave_info_[i].slave->get_slave()->get_position(),
            sdo.index,
            sdo.sub_index,
            buffer,
            sdo.data_size(),
            &abort_code);

      if (ret) {
        RCLCPP_FATAL(
              rclcpp::get_logger("EtherlabMaster"),
              "Failed to download config SDO for module at position %i with Error: %d",
              slave_info_[i].slave->get_slave()->get_position(),
              abort_code);
        return false;
      }
    }
  }

  return true;
}

int EtherlabMaster::upload_slave_sdo(
  uint16_t slave_position, uint16_t index, uint8_t sub_index,
  uint8_t * target, size_t target_size, size_t * result_size, uint32_t * abort_code)
{
  if (activated_) {
    printError(
      "Upload slave SDO. Refusing: this is a blocking mailbox round-trip and must only be "
      "called during the configure phase (after configure_slaves(), before start()), never "
      "while the master is activated — it would stall the real-time cycle.");
    return -1;
  }
  return ecrt_master_sdo_upload(
    master_, slave_position, index, sub_index, target, target_size, result_size, abort_code);
}

ethercat_interface::EcMasterStateInfo EtherlabMaster::get_master_state() const
{
  ethercat_interface::EcMasterStateInfo info;
  info.link_up = master_state_.link_up;
  info.slaves_responding = master_state_.slaves_responding;
  info.al_states = static_cast<uint8_t>(master_state_.al_states);
  return info;
}

ethercat_interface::EcDomainStateInfo EtherlabMaster::get_domain_state(uint32_t domain) const
{
  ethercat_interface::EcDomainStateInfo info;
  const DomainInfo * domain_info = domain_info_.at(domain);
  info.working_counter = domain_info->domain_state.working_counter;
  info.wc_state = static_cast<uint8_t>(domain_info->domain_state.wc_state);
  return info;
}

std::vector<ethercat_interface::EcSlaveStateInfo> EtherlabMaster::get_slave_states() const
{
  std::vector<ethercat_interface::EcSlaveStateInfo> out;
  out.reserve(slave_info_.size());
  for (const SlaveInfo & s : slave_info_) {
    ethercat_interface::EcSlaveStateInfo info;
    info.alias = s.slave->get_slave()->get_alias();
    info.position = s.slave->get_slave()->get_position();
    info.al_state = s.config_state.al_state;
    info.online = s.config_state.online;
    info.operational = s.config_state.operational;
    out.push_back(info);
  }
  return out;
}

  /*int EtherlabMaster::configure_slave(uint16_t slave_position, ethercat_interface::SdoConfigEntry sdo_config, uint32_t *abort_code)
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
  }*/

void EtherlabMaster::registerPDOInDomain(
  std::vector<uint32_t> & channel_indices,
  DomainInfo *domain_info,
  std::shared_ptr<EtherlabSlave> slave)
{
    // expand the size of the domain
  uint32_t num_pdo_regs = channel_indices.size();
  size_t start_index = domain_info->domain_regs.size() - 1;   // empty element at end
  domain_info->domain_regs.resize(domain_info->domain_regs.size() + num_pdo_regs);

    // create a new entry in the domain
  DomainInfo::Entry domain_entry;
  domain_entry.slave = slave;
  domain_entry.num_pdos = num_pdo_regs;
  domain_entry.offset = new uint32_t[num_pdo_regs];
  domain_entry.bit_position = new uint32_t[num_pdo_regs];
  domain_info->entries.push_back(domain_entry);

    // EtherlabSlave::DomainMap domain_map;
    // slave->domains(domain_map);

    // add to array of pdos registrations
  const ec_pdo_entry_info_t *pdo_regs = slave->channels();
  for (size_t i = 0; i < num_pdo_regs; ++i) {
      // create pdo entry in the domain
    ec_pdo_entry_reg_t & pdo_reg = domain_info->domain_regs[start_index + i];
    pdo_reg.alias = slave->get_slave()->get_alias();
    pdo_reg.position = slave->get_slave()->get_position();
    pdo_reg.vendor_id = slave->get_slave()->get_vendor_id();
    pdo_reg.product_code = slave->get_slave()->get_product_id();
    pdo_reg.index = pdo_regs[channel_indices[i]].index;
    pdo_reg.subindex = pdo_regs[channel_indices[i]].subindex;
    pdo_reg.offset = &(domain_entry.offset[i]);
    pdo_reg.bit_position = &(domain_entry.bit_position[i]);

      // print the domain pdo entry
    RCLCPP_INFO(
          rclcpp::get_logger("EthercatDriver"),
          "{ %d, %d, 0x%x, 0x%x, 0x%x, 0x%x }",
          pdo_reg.alias,
          pdo_reg.position,
          pdo_reg.vendor_id,
          pdo_reg.product_code,
          pdo_reg.index,
          static_cast<int>(pdo_reg.subindex));
  }

    // set the last element to null
  ec_pdo_entry_reg_t empty = {0, 0, 0, 0, 0, 0, nullptr, nullptr};
  domain_info->domain_regs.back() = empty;
}

bool EtherlabMaster::start()
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
      printWarning("Start. Failed to register domain PDO entries.");
      return false;
    }
  }
  //  set application time
  //  struct timespec t;
  //  clock_gettime(CLOCK_MONOTONIC, &t);
  //  ecrt_master_application_time(master_, EC_NEWTIMEVAL2NANO(t));

  // activate master
  bool activate_status = ecrt_master_activate(master_);
  if (activate_status) {
    printWarning("Start. Failed to activate ecat master.");
    return false;
  }
  // From here on the master is cyclically exchanging process data: blocking mailbox SDO
  // calls (upload_slave_sdo) are no longer safe, regardless of whether the rest of start()
  // below succeeds.
  activated_ = true;

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
    std::shared_ptr<ethercat_interface::EcSlaveBase> slave = entry.slave->get_slave();
    for (int i = 0; i < entry.num_pdos; ++i) {
      slave->process_data(i, domain_info->domain_pd + entry.offset[i]);
    }
    slave->updateState();
  }


  struct timespec t;

  clock_gettime(CLOCK_MONOTONIC, &t);
  ecrt_master_application_time(master_, EC_NEWTIMEVAL2NANO(t));
  ecrt_master_sync_reference_clock(master_);
  ecrt_master_sync_slave_clocks(master_);

  // send process data
  ecrt_domain_queue(domain_info->domain);
  ecrt_master_send(master_);

  ++update_counter_;
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

    // RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "updated!");

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

  /** stop the control loop.
   */
bool EtherlabMaster::stop()
{
  activated_ = false;
  return true;
}

bool EtherlabMaster::read_process_data()
{
  uint32_t domain = 0;

  // receive process data
  ecrt_master_receive(master_);

  DomainInfo * domain_info = domain_info_.at(domain);
  if (domain_info == NULL) {
    throw std::runtime_error("Null domain info: " + std::to_string(domain));
  }

  ecrt_domain_process(domain_info->domain);

  // Transfer data if configured
  // TODO(@yguel) make transfer per domain ? Quid of transfers across domains ?
  transferAll();

  // check process data state (optional)
  checkDomainState(domain);

  // check for master and slave state change
  if (update_counter_ % check_state_frequency_ == 0) {
    checkMasterState();
    checkSlaveStates();
  }

  // read and write process data
  EtherlabSlave::DomainMap domain_map;
  std::vector<unsigned int> domain_map_;
  for (DomainInfo::Entry & entry : domain_info->entries) {
    std::shared_ptr<ethercat_interface::EcSlaveBase> slave = entry.slave->get_slave();
    entry.slave->domains(domain_map);
    domain_map_ = domain_map.at(0);

    for (auto i = 0; i < entry.num_pdos; ++i) {
      auto index = domain_map_[i];
      slave->process_data(index, domain_info->domain_pd + entry.offset[i]);
    }
    slave->updateState();
  }

  ++update_counter_;
  return true;
}
bool EtherlabMaster::write_process_data()
{
  uint32_t domain = 0;
  DomainInfo * domain_info = domain_info_.at(domain);
  if (domain_info == NULL) {
    throw std::runtime_error("Null domain info: " + std::to_string(domain));
  }

  // read and write process data
  EtherlabSlave::DomainMap domain_map;
  std::vector<unsigned int> domain_map_;
  for (DomainInfo::Entry & entry : domain_info->entries) {
    std::shared_ptr<ethercat_interface::EcSlaveBase> slave = entry.slave->get_slave();
    entry.slave->domains(domain_map);
    domain_map_ = domain_map.at(0);
    for (auto i = 0; i < entry.num_pdos; ++i) {
      auto index = domain_map_[i];
      slave->process_data(index, domain_info->domain_pd + entry.offset[i]);
    }
  }

  struct timespec t;

  clock_gettime(CLOCK_MONOTONIC, &t);
  ecrt_master_application_time(master_, EC_NEWTIMEVAL2NANO(t));
  ecrt_master_sync_reference_clock(master_);
  ecrt_master_sync_slave_clocks(master_);

  // send process data
  ecrt_domain_queue(domain_info->domain);
  ecrt_master_send(master_);


  return true;
}

bool EtherlabMaster::reset()
{
  return true;
}

void EtherlabMaster::checkDomainState(uint32_t domain)
{
  DomainInfo * domain_info = domain_info_.at(domain);
  if (domain_info == NULL) {
    throw std::runtime_error("Null domain info: " + std::to_string(domain));
  }

  ec_domain_state_t ds;
  ecrt_domain_state(domain_info->domain, &ds);

  if (ds.working_counter != domain_info->domain_state.working_counter) {
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Domain: WC %d.", ds.working_counter);
  }
  if (ds.wc_state != domain_info->domain_state.wc_state) {
    RCLCPP_INFO(
        rclcpp::get_logger("EthercatDriver"),
        "Domain: State %s.",
        ds.wc_state == EC_WC_ZERO ? "ZERO" :
      (
        (ds.wc_state == EC_WC_INCOMPLETE) ? "INCOMPLETE" :
        (ds.wc_state == EC_WC_COMPLETE) ? "COMPLETE" : "UNKNOWN"
      )
    );
  }
  domain_info->domain_state = ds;
}

void EtherlabMaster::checkMasterState()
{
  ec_master_state_t ms;
  ecrt_master_state(master_, &ms);

  if (ms.slaves_responding != master_state_.slaves_responding) {
    RCLCPP_WARN(rclcpp::get_logger("EthercatDriver"), "%d slave(s).", ms.slaves_responding);
  }
  if (ms.al_states != master_state_.al_states) {
    RCLCPP_WARN(rclcpp::get_logger("EthercatDriver"), "Master AL states: 0x%02X.", ms.al_states);
  }
  if (ms.link_up != master_state_.link_up) {
    RCLCPP_WARN(rclcpp::get_logger("EthercatDriver"), "Link is %s.", ms.link_up ? "up" : "down");
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
      RCLCPP_WARN(rclcpp::get_logger("EthercatDriver"), "Slave: State 0x%02X.", s.al_state);
    }
    if (s.online != slave.config_state.online) {
      RCLCPP_WARN(
          rclcpp::get_logger(
            "EthercatDriver"), "Slave: %s.", s.online ? "online" : "offline");
    }
    if (s.operational != slave.config_state.operational) {
      RCLCPP_WARN(
          rclcpp::get_logger("EthercatDriver"),
          "Slave: (alias: %d, pos: %d, vendor_id: %d, prod_id: %d) --> %soperational.",
          slave.slave->get_slave()->get_alias(),
          slave.slave->get_slave()->get_position(),
          slave.slave->get_slave()->get_vendor_id(),
          slave.slave->get_slave()->get_product_id(),
          s.operational ? "" : "NOT ");
      slave.slave->get_slave()->set_state_is_operational(s.operational ? true : false);
    }
    slave.config_state = s;
  }
}

  /*void EtherlabMaster::checkDomainInfoValidity(
    const DomainInfo & domain_info,
    const ec_pdo_entry_reg_t & pdo_entry_reg)
  {
    if (nullptr == domain_info.domain_pd) {
      throw std::runtime_error("Domain process data pointer not set.");
    }
    if (nullptr == pdo_entry_reg.offset) {
      throw std::runtime_error("Offset not set in pdo_entry_reg.");
    }
  }*/

void EtherlabMaster::registerTransferInDomain(
  const std::vector<ethercat_interface::EcTransferNet> & transfer_nets)
{
  // Fill in the EcTransferInfo structures

  // For each transfer of each net,
  for (auto & net : transfer_nets) {
    for (auto & transfer : net.transfers) {
      ethercat_interface::EcTransferInfo transfer_info;
      transfer_info.size = transfer.size;
      RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Transfer size: %ld", transfer.size);
        /**
         * For the input and the output of the transfer find
         *   1. the process domain data pointer
         *   2. the offset in the process domain data
         * By iterating over the existing DomainInfo and domain_regs vector
         * to find the ec_pdo_entry_reg_t whose alias, position, index and subindex
         * match the transfer input and output memory entries
         * */
      for (const auto & key_val : domain_info_) {
        const DomainInfo & domain = *(key_val.second);
        for (auto & domain_reg : domain.domain_regs) {
            // Find match for input
          if (domain_reg.alias == transfer.input.alias &&
            domain_reg.position == transfer.input.position &&
            domain_reg.index == transfer.input.index &&
            domain_reg.subindex == transfer.input.subindex)
          {
            transfer_info.input_domain = reinterpret_cast<const void *>(&domain);
              // 3. Compute the pointer arithmetic and store the result in the EcTransferInfo object
            transfer_info.in_ptr = domain.domain_pd + *(domain_reg.offset);
            RCLCPP_INFO(
                rclcpp::get_logger("EthercatDriver"),
                "Transfer input:  esclave position: %d / index: 0x%x / in offset:  %d",
                domain_reg.position,
                domain_reg.index,
                *(domain_reg.offset)
            );
          }
            // Find match for output
          if (domain_reg.alias == transfer.output.alias &&
            domain_reg.position == transfer.output.position &&
            domain_reg.index == transfer.output.index &&
            domain_reg.subindex == transfer.output.subindex)
          {
            transfer_info.output_domain = reinterpret_cast<const void *>(&domain);
              // 3. Compute the pointer arithmetic and store the result in the EcTransferInfo object
            transfer_info.out_ptr = domain.domain_pd + *(domain_reg.offset);
            RCLCPP_INFO(
                rclcpp::get_logger("EthercatDriver"),
                "Transfer output: slave position: %d / index: 0x%x / out offset: %d",
                domain_reg.position,
                domain_reg.index,
                *(domain_reg.offset)
            );
          }
        }
      }

        // Record the transfer
      transfers_.push_back(transfer_info);
    }
  }
}

void EtherlabMaster::transferAll()
{
    // Proceed to the transfer of all the data declared in transfers_.
  for (auto & transfer : transfers_) {
      // Copy the data from the input to the output
    memcpy(transfer.out_ptr, transfer.in_ptr, transfer.size);
  }
}

}  // namespace ethercat_master
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ethercat_master::EtherlabMaster, ethercat_interface::EcMasterBase)
