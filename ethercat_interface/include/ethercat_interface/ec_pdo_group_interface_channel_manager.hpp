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

#ifndef ETHERCAT_INTERFACE__EC_PDO_GROUP_INTERFACE_CHANNEL_MANAGER_HPP_
#define ETHERCAT_INTERFACE__EC_PDO_GROUP_INTERFACE_CHANNEL_MANAGER_HPP_

#include <utility>  // for std::pair
#include <limits>
#include <string>
#include <vector>
#include "ethercat_interface/ec_pdo_channel_manager.hpp"

namespace ethercat_interface
{

struct InterfaceDataWithAddrOffset : public InterfaceData
{
  InterfaceDataWithAddrOffset()
  : InterfaceData(),
    addr_offset(0) {}

  explicit InterfaceDataWithAddrOffset(const InterfaceData & data)
  : InterfaceData(data),
    addr_offset(0) {}

  InterfaceDataWithAddrOffset(const InterfaceData & data, size_t addr_offset)
  : InterfaceData(data),
    addr_offset(addr_offset) {}

  size_t addr_offset = 0;
  uint8_t bits = 0;
  uint8_t data_type_idx = 0;
};

/**
 * @brief Class to manage a PDO channel corresponding to a group of interfaces
 */
class EcPdoGroupInterfaceChannelManager : public EcPdoChannelManager
{
public:
  EcPdoGroupInterfaceChannelManager();
  EcPdoGroupInterfaceChannelManager(const EcPdoGroupInterfaceChannelManager &) = delete;
  ~EcPdoGroupInterfaceChannelManager();

public:
//=======================
/** @name Setup methods
   * @brief Methods to setup the PDO channel manager
   * @{
   */

/// @brief Load the channel configuration from a YAML node
  bool load_from_config(YAML::Node channel_config);

/** @} */    // end of Setup methods
//=======================

public:
//=======================
/** @name Data exchange methods
   * @brief Methods to read and write data from/to the PDO
   * @{
   */

  /** @brief Read the data from the PDO applying data mask, factor and offset
   * @param i The index of the interface to read
   * @param domain_address The pdo start address in the domain to read from, the read address will be domain_address + v_data[i].addr_offset
   * @returns The value read from the domain
   */
  double ec_read(uint8_t * domain_address, size_t i = 0);

  /// @brief Perform an ec_read and update the state interface
  void ec_read_to_interface(uint8_t * domain_address);

  /** @brief Write the value to the PDO applying data mask, factor and offset
   * @param i The index of the interface to write
   * @param domain_address The pdo start address in the domain to write into, the write address will be domain_address + v_data[i].addr_offset
   * @param value The value to write
   */
  void ec_write(uint8_t * domain_address, double value, size_t i = 0);

  /// @brief Perform an ec_write and update the command interface
  void ec_write_from_interface(uint8_t * domain_address);

/** @} */    // < end of Data exchange methods
//=======================

public:
  std::vector<InterfaceDataWithAddrOffset> v_data;

public:
  inline
  size_t number_of_interfaces() const
  {
    return v_data.size();
  }

  inline
  size_t number_of_managed_interfaces() const
  {
    return managed_.size();
  }

  void setup_managed_interfaces();

  std::string interface_name(size_t i = 0) const;

  std::pair<bool, size_t> is_interface_managed(std::string name) const;

  size_t channel_state_interface_index(const std::string & name) const;

  size_t channel_command_interface_index(const std::string & name) const;

  std::string data_type(size_t i = 0) const;

  size_t state_interface_index(size_t i = 0) const;
  size_t command_interface_index(size_t i = 0) const;

public:
  inline
  void set_state_interface_index(const std::string & interface_name, size_t index)
  {
    size_t i = channel_state_interface_index(interface_name);
    interface_ids_.at(i) = index;
  }

  inline
  void set_command_interface_index(const std::string & interface_name, size_t index)
  {
    size_t i = channel_command_interface_index(interface_name);
    interface_ids_.at(i) = index;
  }

  inline
  bool is_interface_defined(size_t i) const
  {
    return std::numeric_limits<size_t>::max() != interface_ids_.at(i);
  }

  inline
  bool is_state_interface_defined(size_t i) const
  {
    return is_interface_defined(i) && !is_command_interface_.at(i);
  }

  inline
  bool is_command_interface_defined(size_t i) const
  {
    return is_interface_defined(i) && is_command_interface_.at(i);
  }

  inline
  bool has_interface_name(size_t i = 0) const
  {
    return 0 != interface_name_ids_.at(i);
  }

  inline bool has_state_interface_name(size_t i = 0) const
  {
    return has_interface_name(i) && !is_command_interface_.at(i);
  }

  inline bool has_command_interface_name(size_t i = 0) const
  {
    return has_interface_name(i) && is_command_interface_.at(i);
  }

public:
  inline
  InterfaceData & data(size_t i = 0)
  {
    return v_data.at(i);
  }

  inline
  const InterfaceData & data(size_t i = 0) const
  {
    return v_data.at(i);
  }

protected:
/** @brief Create the necessary allocations to add a new interface */
  void allocate_for_new_interface();

/** @brief Add a state interface named name
 * @details If the interface is already present, the function does nothing and
 *  returns the index of the interface, otherwise it adds the interface and
 *  returns its index
 *
 * @param[in] name the name of the interface to add
 * @returns the index for the added interface in all the vectors
 * (interface_ids_, interface_name_ids_, read_functions_, data,
 * write_functions_)
 *
 */
  size_t add_state_interface(const std::string & name);

/** @brief Add a data without interface
 * @details The function adds a data without interface and returns the index
 * of the data in all the vectors (interface_ids_, interface_name_ids_,
 * read_functions_, data, write_functions_)
 * @returns the index for the added data in all the vectors
 */
  size_t add_data_without_interface();

/** @brief Add a command interface named name
 * @details If the interface is already present, the function does nothing and
 * returns the index of the interface, otherwise it adds the interface and
 * returns its index
 *
 * @param[in] name the name of the interface to add
 * @returns the index for the added interface in all the vectors
 * (command_interface_ids_, command_interface_name_ids_, write_functions_, data
 * but also state_interface_ids_, state_interface_name_ids_,
 * read_functions_)
 *
 */
  size_t add_command_interface(const std::string & name);

protected:
  /** @brief Indices of the state/or command interfaces in the ros2 control
   * state interface or command interface vector
   * @details If the index is not set, the value is std::numeric_limits<size_t>::max()
   * To know which type of interface vector the index refers to, it is necessary to
   * look into the is_command_interface_ vector.
   */
  std::vector<size_t> interface_ids_;

  /** @brief Store a boolean indicating if the interface is a command interface
   * or a state interface
  */
  std::vector<bool> is_command_interface_;

  /** @brief Stores the function used to read the data from the EtherCAT
   * frame for each interface */
  std::vector<SingleReadFunctionType> read_functions_;

  /** @brief Stores the function used to write the data to the EtherCAT
   * frame for each interface */
  std::vector<SingleWriteFunctionType> write_functions_;

  /** @brief Stores the index to the name of the interfaces
   * @details The index of the name of the interfaces is stored in the same
   * order as the interfaces in the data vector. To know if the interface is
   * a command or a state interface, it is necessary to look into the
   * is_command_interface_ vector.
   * If the interface is a command interface then the index refers to a name in
   * the all_command_interface_names vector, otherwise it refers to a name in the
   * all_state_interface_names vector.
   * If the interface is not defined, the value is 0
  */
  std::vector<size_t> interface_name_ids_;

  /** @brief Indices of the interfaces that are either state or command interfaces
   * @details The indices are stored in the same order as the interfaces in the
   * data vector. It stores only the indices of the interfaces index idx where
   * interface_name_ids_[idx] is not 0.
   */
  std::vector<size_t> managed_;
};

}  // < namespace ethercat_interface


#endif  // ETHERCAT_INTERFACE__EC_PDO_GROUP_INTERFACE_CHANNEL_MANAGER_HPP_
