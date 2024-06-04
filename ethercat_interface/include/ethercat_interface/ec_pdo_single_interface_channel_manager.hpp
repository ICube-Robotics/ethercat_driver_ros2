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

#ifndef ETHERCAT_INTERFACE__EC_PDO_SINGLE_INTERFACE_CHANNEL_MANAGER_HPP_
#define ETHERCAT_INTERFACE__EC_PDO_SINGLE_INTERFACE_CHANNEL_MANAGER_HPP_

#include <utility>  // for std::pair
#include <limits>
#include <string>
#include <vector>
#include "ethercat_interface/ec_pdo_channel_manager.hpp"

namespace ethercat_interface
{

/**
 * @brief Class to manage a PDO channel corresponding to a single interface
 */
class EcPdoSingleInterfaceChannelManager : public EcPdoChannelManager, public InterfaceData
{
public:
  EcPdoSingleInterfaceChannelManager();
  EcPdoSingleInterfaceChannelManager(const EcPdoSingleInterfaceChannelManager &) = delete;
  ~EcPdoSingleInterfaceChannelManager();

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

  /// @brief Read the data from the PDO applying data mask, factor and offset
  double ec_read(uint8_t * domain_address, size_t i = 0);

  /// @brief Perform an ec_read and update the state interface
  void ec_read_to_interface(uint8_t * domain_address);

  /// @brief Write the value to the PDO applying data mask, factor and offset
  void ec_write(uint8_t * domain_address, double value, size_t i = 0);

  /// @brief Perform an ec_write and update the command interface
  void ec_write_from_interface(uint8_t * domain_address);

/** @} */    // < end of Data exchange methods
//=======================

public:
  inline
  size_t number_of_interfaces() const
  {
    return 1;
  }

  inline
  size_t number_of_managed_interfaces() const
  {
    return (state_interface_name_idx_ + command_interface_name_idx_) > 0;
  }

  inline
  void setup_managed_interfaces() {}

  inline std::string interface_name(size_t i = 0) const
  {
    if (0 == i) {
      if (has_command_interface_name()) {
        return all_command_interface_names[command_interface_name_idx_];
      } else {
        if (has_state_interface_name()) {
          return all_state_interface_names[state_interface_name_idx_];
        } else {
          return "null";
        }
      }
    }
    throw std::out_of_range(
            "EcPdoSingleInterfaceChannelManager::interface_name "
            "unknown interface index : must be 0 (instead of " +
            std::to_string(i) + ")");
  }

  inline
  std::string data_type(size_t i = 0) const
  {
    if (0 == i) {
      return id_and_bits_to_type(data_type_idx_, bits_);
    } else {
      throw std::out_of_range(
              "EcPdoSingleInterfaceChannelManager::data_type unknown "
              "interface index : must be 0 (instead of " +
              std::to_string(i) + ")");
    }
  }

  /** @brief Test if an interface named «name» is managed and returns its index among the
   * interfaces managed by the PDO channel.
   * @param name the name of the interface to test
   * @return a pair with a boolean indicating if the interface is managed and the index
   * of the interface among the interfaces managed by the PDO channel
   */
  std::pair<bool, size_t> is_interface_managed(std::string name) const;

public:
  inline
  void set_state_interface_index(const std::string & /*interface_name*/, size_t index)
  {
    state_interface_index_ = index;
  }

  inline
  void set_command_interface_index(const std::string & /*interface_name*/, size_t index)
  {
    command_interface_index_ = index;
  }

  inline
  bool has_state_interface_name(size_t /*i*/ = 0) const
  {
    return 0 != state_interface_name_idx_;
  }

  inline
  bool has_command_interface_name(size_t /*i*/ = 0) const
  {
    return 0 != command_interface_name_idx_;
  }

  inline
  bool has_interface_name(size_t /*i*/) const
  {
    return has_state_interface_name() || has_command_interface_name();
  }

  inline
  bool is_state_interface_defined() const
  {
    return std::numeric_limits<size_t>::max() != state_interface_index_;
  }

  inline
  bool is_command_interface_defined() const
  {
    return std::numeric_limits<size_t>::max() != command_interface_index_;
  }

  inline size_t state_interface_index(size_t /*i*/) const
  {
    return state_interface_index_;
  }

  inline size_t command_interface_index(size_t /*i*/) const
  {
    return command_interface_index_;
  }

public:
  inline InterfaceData & data(size_t i = 0)
  {
    if (0 == i) {
      return *this;
    }
    throw std::out_of_range(
            "EcPdoSingleInterfaceChannelManager::data unknown "
            "interface index : must be 0 (instead of " +
            std::to_string(i) + ")");
  }

  inline const InterfaceData & data(size_t i = 0) const
  {
    if (0 == i) {
      return *this;
    }
    throw std::out_of_range(
            "EcPdoSingleInterfaceChannelManager::data unknown "
            "interface index : must be 0 (instead of " +
            std::to_string(i) + ")");
  }

protected:
  /** @brief Index of the state interface in the ros2 control state interface vector
   * @details If the index is not set, the value is std::numeric_limits<size_t>::max()
   */
  size_t state_interface_index_ = std::numeric_limits<size_t>::max();
  /** @brief Index of the command interface in the ros2 control state interface vector
   * @details If the index is not set, the value is std::numeric_limits<size_t>::max()
   */
  size_t command_interface_index_ = std::numeric_limits<size_t>::max();
  SingleReadFunctionType read_function_ = nullptr;
  SingleWriteFunctionType write_function_ = nullptr;
  size_t state_interface_name_idx_ = 0;
  size_t command_interface_name_idx_ = 0;
};

}  // < namespace ethercat_interface


#endif  // ETHERCAT_INTERFACE__EC_PDO_SINGLE_INTERFACE_CHANNEL_MANAGER_HPP_
