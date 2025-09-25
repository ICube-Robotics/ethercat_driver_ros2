// Copyright 2023-2024 ICUBE Laboratory, University of Strasbourg
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
// Author: Maciej Bednarczyk (macbednarczyk@gmail.com)
// Author: Manuel YGUEL (yguel.robotics@gmail.com)


#ifndef ETHERCAT_INTERFACE__EC_PDO_CHANNEL_MANAGER_HPP_
#define ETHERCAT_INTERFACE__EC_PDO_CHANNEL_MANAGER_HPP_

#include <yaml-cpp/yaml.h>
#include <ecrt.h>

#include <string>
#include <vector>
#include <limits>
#include <stdexcept>
#include <utility>
#include <sstream>

namespace ethercat_interface
{

enum PdoType
{
  RPDO = 0,  // < Receive PDO, Master out to Slave in (MoSi)
  TPDO = 1  // < Transmit PDO, Master in from Slave out (MiSo)
};

/** @brief Global table that stores all the names of the known types */
extern const std::vector<std::string> ec_pdo_channel_data_types;

/** @brief Global table that stores all the number of bits of the known types */
extern const std::vector<uint8_t> ec_pdo_channel_data_bits;

/** @brief Returns the index of a data type in the table of types */
size_t typeIdx(const std::string & type);

/** @brief Returns the number of bits associated with a data type */
uint8_t type2bits(const std::string & type);

/** @brief Returns the type name */
std::string id_and_bits_to_type(size_t i, uint8_t bits);

/** @brief Check if the definition of a type is correct
 * @param[in] type the name of the type to check
 * @param[in] mask the mask associated with the type to check the compatibility
 * @return true if the type is correct, false otherwise
 */
bool check_type(const std::string & type, uint8_t mask);

/** @brief Global table that stores all the names of the recorded state interfaces */
extern std::vector<std::string> all_state_interface_names;

/** @brief Global table that stores all the names of the recorded command interfaces */
extern std::vector<std::string> all_command_interface_names;

/** @brief The type of the functions used to read from EtherCAT frame and return a double
 * to be compatible with a ROS2 control state interface */
typedef double (* SingleReadFunctionType)(uint8_t * domain_address, uint8_t data_mask);

/** @brief Global table that stores each read function associated with a data type */
extern const SingleReadFunctionType ec_pdo_single_read_functions[];

/** @brief The type of the functions used to write to EtherCAT frame from a double
 * to be compatible with a ROS2 control command interface */
typedef void (* SingleWriteFunctionType)(uint8_t * domain_address, double value, uint8_t data_mask);

/** @brief Global table that stores each write function associated with a data type */
extern const SingleWriteFunctionType ec_pdo_single_write_functions[];

struct InterfaceData
{
  bool override_command = false;
  uint8_t mask = 255;
  double default_value = std::numeric_limits<double>::quiet_NaN();
  /** last_value stores either:
   * - the last read value modified by mask, factor and offset
   * - the last written value modified by mask, factor and offset
   * */
  double last_value = std::numeric_limits<double>::quiet_NaN();
  double factor = 1;
  double offset = 0;
};


/**
 * @brief Virtual class for managing a single PDO channel
 * @details The EcPdoChannelManager class is used to manage a single PDO
 * channel. It is used to read and write data to the channel, and to update
 * the ROS2 control interfaces (state and command).
 * The PDO can correspond to a single interface, or to a group of interfaces.
 * Hence the specializations of this class to manage these two cases.
 * The most common case is the single interface case, where the PDO corresponds
 * to a single interface. It is less common to have a group of interfaces,
 * but it is still possible and useful (see the example below).
 * @example Single interface example: a joint position PDO.
 * The channel manager will read the joint position from the PDO and write the
 * joint position to the PDO. The joint position will be updated in the state
 * interface, and the joint position will be read from the command interface.
 * The data in the channel may be an integer corresponding to coder ticks.
 * The coder ticks will be converted to radians and stored in the state
 * interface. The joint position in radians will be read from the command
 * interface and converted to coder ticks and written to the PDO.
 * @example Group of interfaces example: a set of states can be encoded in an
 * octet, each bit corresponding to a state. It is particularly useful from a
 * coding perspective to have one interface per state. The channel manager will
 * read the octet from the PDO and update a group of state interfaces
 * corresponding to each bit in the octet related to a state. The channel
 * manager will also read the group of command interfaces and write the single
 * octet to the PDO that encodes all the states.
 * @note More generally the EcGroupInterfacePdoChannelManager class is used
 * when some specific data encoding is used to represent a group of
 * interfaces in the EtherCAT memory frame but associated to a single PDO.
 */
class EcPdoChannelManager
{
public:
  EcPdoChannelManager();
  EcPdoChannelManager(const EcPdoChannelManager &) = delete;
  virtual ~EcPdoChannelManager() = 0;

public:
//=======================
/** @name Setup methods
   * @brief Methods to setup the PDO channel manager
   * @{
   */

  /** @brief Record the pointers to the state and command interfaces */
  void setup_interface_ptrs(
    std::vector<double> * state_interface,
    std::vector<double> * command_interface);

  /** @brief Load the channel configuration from a YAML node */
  virtual bool load_from_config(YAML::Node channel_config) = 0;

/** @} */  // < end of Setup methods
//=======================

public:
//=======================
/** @name Data exchange methods
   * @brief Methods to read and write data from/to the PDO
   * @{
   */
  virtual double ec_read(uint8_t * domain_address, size_t i = 0) = 0;

  /// @brief Perform an ec_read and update the state interface
  virtual void ec_read_to_interface(uint8_t * domain_address) = 0;

  virtual void ec_write(uint8_t * domain_address, double value, size_t i = 0) = 0;

  /// @brief Perform an ec_write and update the command interface
  virtual void ec_write_from_interface(uint8_t * domain_address) = 0;

  /// @brief Update the state and command interfaces
  virtual void ec_update(uint8_t * domain_address);

/** @} */  // < end of Data exchange methods
//=======================

public:
  /** @brief Get the PDO entry info as it should be recorded by the master*/
  ec_pdo_entry_info_t get_pdo_entry_info();

public:
  PdoType pdo_type;
  uint16_t index;
  uint8_t sub_index;

  inline
  std::string index_hex_str() const
  {
    std::stringstream ss;
    ss << "0x" << std::hex << index;
    return ss.str();
  }

  inline
  std::string sub_index_hex_str() const
  {
    std::stringstream ss;
    ss << "0x" << std::hex << sub_index;
    return ss.str();
  }

  /** @brief Get the number of bits in the PDO */
  inline uint8_t pdo_bits() const {return bits_;}

  /** @brief Get the string describing the data type of the PDO */
  inline
  std::string pdo_data_type() const
  {
    return id_and_bits_to_type(data_type_idx_, bits_);
  }

  /** @brief Get the string describing the type of the data associated with interface i
   * @param i The index of the interface in the group of interfaces managed by the PDO channel. In case of a single interface, i must be 0.
  */
  virtual std::string data_type(size_t i = 0) const = 0;

  /** @brief Get the data */
  virtual InterfaceData & data(size_t i = 0) = 0;

  /** @brief Get the data */
  virtual const InterfaceData & data(size_t i = 0) const = 0;

  bool allow_ec_write = true;  // < Is the PDO channel writable ?

public:
  bool skip = false;  // < Skip the PDO channel in ? TODO(@yguel@unistra.fr)

public:
  virtual size_t number_of_interfaces() const = 0;
  virtual void setup_managed_interfaces() = 0;
  virtual size_t number_of_managed_interfaces() const = 0;
  virtual std::string interface_name(size_t i = 0) const = 0;
  virtual std::pair<bool, size_t> is_interface_managed(std::string interface_name) const = 0;
  virtual void set_state_interface_index(const std::string & interface_name, size_t index) = 0;
  virtual void set_command_interface_index(const std::string & interface_name, size_t index) = 0;
  virtual size_t state_interface_index(size_t i = 0) const = 0;
  virtual size_t command_interface_index(size_t i = 0) const = 0;
  virtual bool has_interface_name(size_t i = 0) const = 0;
  virtual bool has_state_interface_name(size_t i = 0) const = 0;
  virtual bool has_command_interface_name(size_t i = 0) const = 0;

protected:
  uint8_t bits_;  // < Number of bits declared in the PDO
  uint8_t data_type_idx_;  // < Index to the table of types to infer the data type

protected:
  std::vector<double> * command_interface_ptr_;
  std::vector<double> * state_interface_ptr_;
};

}  // < namespace ethercat_interface
#endif  // ETHERCAT_INTERFACE__EC_PDO_CHANNEL_MANAGER_HPP_
