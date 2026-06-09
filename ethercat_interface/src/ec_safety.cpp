// Copyright 2024 ICUBE Laboratory, University of Strasbourg
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
// Author: Manuel YGUEL (yguel.robotics@gmail.com)

#include "ethercat_interface/ec_safety.hpp"
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
#include <bitset>

namespace ethercat_interface
{

void EcSafety::checkDomainInfoValidity(
  const DomainInfo & domain_info,
  const ec_pdo_entry_reg_t & pdo_entry_reg)
{
  if (nullptr == domain_info.domain_pd) {
    throw std::runtime_error("Domain process data pointer not set.");
  }
  if (nullptr == pdo_entry_reg.offset) {
    throw std::runtime_error("Offset not set in pdo_entry_reg.");
  }
}

void EcSafety::registerTransferInDomain(const std::vector<EcSafetyNet> & safety_nets)
{
  // Fill in the EcTransferInfo structures

  // For each transfer of each net,
  for (auto & net : safety_nets) {
    for (auto & transfer : net.transfers) {
      EcTransferInfo transfer_info;
      transfer_info.size = transfer.size;
      std::cout << "Size: " << transfer_info.size << std::endl;
      /**
       * For the input and the output of the transfer find
       *   1. the process domain data pointer
       *   2. the offset in the process domain data
       * By iterating over the existing DomainInfo and domain_regs vector
       * to find the ec_pdo_entry_reg_t whose alias, position, index and subindex
       * match the transfer input and output memory entries
       */
      for (const auto & key_val : domain_info_) {
        const DomainInfo & domain = *(key_val.second);
        for (auto & domain_reg : domain.domain_regs) {
          // Find match for input
          if (domain_reg.alias == transfer.input.alias &&
            domain_reg.position == transfer.input.position &&
            domain_reg.index == transfer.input.index &&
            domain_reg.subindex == transfer.input.subindex)
          {
            transfer_info.input_domain = &domain;
            // 3. Compute the pointer arithmetic and store the result in the EcTransferInfo object
            transfer_info.in_ptr = domain.domain_pd + *(domain_reg.offset);
            std::cout << "esclave position: " << domain_reg.position << std::endl;
            std::ios oldState(nullptr);
            oldState.copyfmt(std::cout);
            std::cout << "index: 0x" << std::hex << domain_reg.index << std::endl;
            std::cout.copyfmt(oldState);
            std::cout << "in offset: " << *(domain_reg.offset) << std::endl;
            std::cout << std::endl;
          }
          // Find match for output
          if (domain_reg.alias == transfer.output.alias &&
            domain_reg.position == transfer.output.position &&
            domain_reg.index == transfer.output.index &&
            domain_reg.subindex == transfer.output.subindex)
          {
            transfer_info.output_domain = &domain;
            // 3. Compute the pointer arithmetic and store the result in the EcTransferInfo object
            transfer_info.out_ptr = domain.domain_pd + *(domain_reg.offset);
            std::cout << "esclave position: " << domain_reg.position << std::endl;
            std::ios oldState(nullptr);
            oldState.copyfmt(std::cout);
            std::cout << "index: 0x" << std::hex << domain_reg.index << std::endl;
            std::cout.copyfmt(oldState);
            std::cout << "out offset: " << *(domain_reg.offset) << std::endl;
            std::cout << std::endl;
          }
        }
      }

      // Record the transfer
      transfers_.push_back(transfer_info);
    }
  }
}

void EcSafety::printMemoryFrames(std::ostream & os)
{
  for (auto & kv : domain_info_) {
    os << "Domain: " << kv.first << std::endl;
    auto & d = kv.second;
    size_t size = ecrt_domain_size(d->domain);
    // Display the memory
    for (size_t i = 0; i < size; i++) {
      os << std::hex << static_cast<int>(d->domain_pd[i]) << " ";
    }
    os << std::endl;
  }
}

uint8_t * EcSafety::getMemoryStart(
  const uint16_t position,
  const uint16_t index,
  const uint16_t subindex)
{
  for (auto & kv : domain_info_) {
    auto & d = kv.second;
    for (auto & reg : d->domain_regs) {
      if (reg.position == position && reg.index == index && reg.subindex == subindex) {
        return d->domain_pd + *(reg.offset);
      }
    }
  }
  return nullptr;
}

void EcSafety::printMemoryFrame(
  const uint16_t position,
  const uint16_t index,
  const uint16_t subindex,
  const size_t n,
  bool binary,
  std::ostream & os)
{
  uint8_t * pointer = getMemoryStart(position, index, subindex);
  if (pointer != nullptr) {
    for (size_t i = 0; i < n; i++) {
      if (binary) {
        os << std::bitset<8>(pointer[i]) << " ";
      } else {
        os << std::hex << static_cast<int>(pointer[i]) << " ";
      }
    }
    os << std::endl;
  }
}

void EcSafety::transferAll()
{
  // Proceed to the transfer of all the data declared in transfers_.
  for (auto & transfer : transfers_) {
    // Copy the data from the input to the output
    memcpy(transfer.out_ptr, transfer.in_ptr, transfer.size);
  }
}

void EcSafety::update(rclcpp::Logger logger, uint32_t domain)
{
  // receive process data
  ecrt_master_receive(master_);

  DomainInfo * domain_info = domain_info_.at(domain);
  if (domain_info == NULL) {
    throw std::runtime_error("Null domain info: " + std::to_string(domain));
  }

  ecrt_domain_process(domain_info->domain);

  // TODO(yguel) make transfer per domain ? Quid of transfers across domains ?
  transferAll();

  // check process data state (optional)
  checkDomainState(logger, domain);

  // check for master and slave state change
  if (update_counter_ % check_state_frequency_ == 0) {
    checkMasterState();
    checkSlaveStates();
  }


  for (DomainInfo::Entry & entry : domain_info->entries) {
    // read and write process data
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

inline std::string word2Str(uint16_t word)
{
  std::stringstream ss;
  ss << "0b" << std::bitset<8>(word);
  return ss.str();
}

void EcSafety::readData(rclcpp::Logger logger, uint32_t domain)
{
  // receive process data
  ecrt_master_receive(master_);

  DomainInfo * domain_info = domain_info_.at(domain);
  if (domain_info == NULL) {
    throw std::runtime_error("Null domain info: " + std::to_string(domain));
  }

  ecrt_domain_process(domain_info->domain);

  // TODO(yguel) make transfer per domain ? Quid of transfers across domains ?
  transferAll();

  // check process data state (optional)
  checkDomainState(logger, domain);

  // check for master and slave state change
  if (update_counter_ % check_state_frequency_ == 0) {
    checkMasterState();
    checkSlaveStates();
  }

  // read and write process data
  for (DomainInfo::Entry & entry : domain_info->entries) {
    // read and write process data
    for (int i = 0; i < entry.num_pdos; ++i) {
      (entry.slave)->processData(i, domain_info->domain_pd + entry.offset[i]);
    }
  }

  ++update_counter_;
}

EcSafety::EcSafety(const unsigned int master)
: EcMaster(master)
{
}

EcSafety::~EcSafety()
{
}

}  // namespace ethercat_interface
