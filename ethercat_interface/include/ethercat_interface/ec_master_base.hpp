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

#include <string>
#include <memory>
#include <vector>
#include "ethercat_interface/ec_slave_base.hpp"
#include "ethercat_interface/ec_transfer.hpp"
#include "rclcpp/rclcpp.hpp"


namespace ethercat_interface
{

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

  /** \brief add a slave device to the master */
  virtual bool add_slave(std::shared_ptr<EcSlaveBase> slave) = 0;

  /** \brief configure slave using SDO */
  virtual bool configure_slaves() = 0;

  virtual bool init(std::string iface) = 0;

  virtual bool start() = 0;

  virtual bool stop() = 0;

  virtual bool reset() = 0;

  virtual bool spin_slaves_until_operational() = 0;

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
