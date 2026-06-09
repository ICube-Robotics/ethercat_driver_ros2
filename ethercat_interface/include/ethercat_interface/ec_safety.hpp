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

#ifndef ETHERCAT_INTERFACE__EC_SAFETY_HPP_
#define ETHERCAT_INTERFACE__EC_SAFETY_HPP_

#include <ecrt.h>

#include <string>
#include <vector>
#include <rclcpp/logger.hpp>

#include "ethercat_interface/ec_master.hpp"
#include "ethercat_interface/ec_transfer.hpp"

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

class EcSafetyNet
{
public:
  std::string name;                    //< safety net name
  std::string master;                  //< safety master
  std::vector<EcTransferEntry> transfers;   //< safety data transfers

public:
  void reset(const std::string & new_name)
  {
    name = new_name;
    master = "";
    transfers.clear();
  }
};

class EcSafety : public EcMaster
{
public:
  explicit EcSafety(const unsigned int master = 0);
  virtual ~EcSafety();

public:
  /** @brief Fill in the EcTransferInfo structures
  *
  * @param safety_nets Safety nets
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
  void registerTransferInDomain(const std::vector<EcSafetyNet> & safety_nets);

  /** @brief Proceed to the transfer of all the data declared in transfers_.
   */
  void transferAll();

  /** perform one EtherCAT cycle, passing the domain to the slaves */
  void update(rclcpp::Logger logger, uint32_t domain = 0);

  void readData(rclcpp::Logger logger, uint32_t domain = 0);
  // TODO(yguel) investigate readData and writeData specifications
  // both functions are called but process all the data in the domain
  //  not only the data rpdos for the write and the tpdos for the read
  //  why ?
  // void writeData(uint32_t domain = 0);

protected:
  /** @brief Output the memory content of the all the domains
   * (available for pedagogic and debug purposes)
   *
   * @param[out] os Output stream
  */
  void printMemoryFrames(std::ostream & os = std::cout);

  /** @brief Get pointer on memory frame for a certain point
   * in the frame defined by a slave position, an index and a subindex
   */
  uint8_t * getMemoryStart(
    const uint16_t position,
    const uint16_t index,
    const uint16_t subindex);

  /** @brief Output memory n bytes of memory from a certain point in
   * the frame defined by a slave position, an index, subindex */
  void printMemoryFrame(
    const uint16_t position,
    const uint16_t index,
    const uint16_t subindex,
    const size_t n,
    bool binary = false,
    std::ostream & os = std::cout);

  /** @brief Check the validity of the domain info and the ec_pdo_entry_reg_t
   * and throw an exception if not valid.
   *
   * @param domain_info Domain info
   * @param pdo_entry_reg PDO entry registration
   *
   * @throw std::runtime_error if domain_info or pdo_entry_reg is not valid
  */
  void checkDomainInfoValidity(
    const DomainInfo & domain_info,
    const ec_pdo_entry_reg_t & pdo_entry_reg);

protected:
  std::vector<EcTransferInfo> transfers_;
};

}  // namespace ethercat_interface

#endif  // ETHERCAT_INTERFACE__EC_SAFETY_HPP_
