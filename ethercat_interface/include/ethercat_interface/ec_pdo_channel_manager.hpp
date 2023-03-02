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
// Author: Maciej Bednarczyk (macbednarczyk@gmail.com)

#ifndef ETHERCAT_INTERFACE__EC_PDO_CHANNEL_MANAGER_HPP_
#define ETHERCAT_INTERFACE__EC_PDO_CHANNEL_MANAGER_HPP_

#include <ecrt.h>
#include <string>
#include <vector>

namespace ethercat_interface
{
enum PdoType
{
  RPDO = 0,
  TPDO = 1
};

class EcPdoChannelManager
{
public:
  EcPdoChannelManager() {}
  ~EcPdoChannelManager() {}
  void setup_interface_ptrs(
    std::vector<double> * state_interface,
    std::vector<double> * command_interface)
  {
    command_interface_ptr_ = command_interface;
    state_interface_ptr_ = state_interface;
  }

  ec_pdo_entry_info_t get_pdo_entry_info() {return {index, sub_index, type2bits(data_type)};}

  double ec_read(uint8_t * domain_address)
  {
    if (data_type == "uint8") {
      return static_cast<double>(EC_READ_U8(domain_address));
    } else if (data_type == "int8") {
      last_value = static_cast<double>(EC_READ_S8(domain_address));
    } else if (data_type == "uint16") {
      last_value = static_cast<double>(EC_READ_U16(domain_address));
    } else if (data_type == "int16") {
      last_value = static_cast<double>(EC_READ_S16(domain_address));
    } else if (data_type == "uint32") {
      last_value = static_cast<double>(EC_READ_U32(domain_address));
    } else if (data_type == "int32") {
      last_value = static_cast<double>(EC_READ_S32(domain_address));
    } else if (data_type == "uint64") {
      last_value = static_cast<double>(EC_READ_U64(domain_address));
    } else if (data_type == "int64") {
      last_value = static_cast<double>(EC_READ_S64(domain_address));
    } else if (data_type == "test") {
      last_value = 42;
    }
    return last_value;
  }

  void ec_write(uint8_t * domain_address, double value)
  {
    if (data_type == "uint8") {
      EC_WRITE_U8(domain_address, static_cast<uint8_t>(value));
    } else if (data_type == "int8") {
      EC_WRITE_S8(domain_address, static_cast<int8_t>(value));
    } else if (data_type == "uint16") {
      EC_WRITE_U16(domain_address, static_cast<uint16_t>(value));
    } else if (data_type == "int16") {
      EC_WRITE_S16(domain_address, static_cast<int16_t>(value));
    } else if (data_type == "uint32") {
      EC_WRITE_U32(domain_address, static_cast<uint32_t>(value));
    } else if (data_type == "int32") {
      EC_WRITE_S32(domain_address, static_cast<int32_t>(value));
    } else if (data_type == "uint64") {
      EC_WRITE_U64(domain_address, static_cast<uint64_t>(value));
    } else if (data_type == "int64") {
      EC_WRITE_S64(domain_address, static_cast<int64_t>(value));
    }
    last_value = value;
  }

  void ec_update(uint8_t * domain_address)
  {
    // update state interface
    if (pdo_type == TPDO) {
      ec_read(domain_address);
      if (interface_index >= 0) {
        state_interface_ptr_->at(interface_index) = last_value;
      }
    } else if (pdo_type == RPDO && allow_ec_write) {
      if (interface_index >= 0 && !std::isnan(command_interface_ptr_->at(interface_index))) {
        ec_write(domain_address, command_interface_ptr_->at(interface_index));
      } else {
        ec_write(domain_address, default_value);
      }
    }
  }

  PdoType pdo_type;
  uint16_t index;
  uint8_t sub_index;
  std::string data_type;
  std::string interface_name;
  uint32_t data_mask;
  double default_value;
  int interface_index = -1;
  double last_value;
  bool allow_ec_write = true;

private:
  std::vector<double> * command_interface_ptr_;
  std::vector<double> * state_interface_ptr_;

  uint8_t type2bits(std::string type)
  {
    if (type == "int8" || type == "uint8") {
      return 8;
    } else if (type == "int16" || type == "uint16") {
      return 16;
    } else if (type == "int32" || type == "uint32") {
      return 32;
    } else if (type == "int64" || type == "uint64") {
      return 64;
    }
  }
};

}  // namespace ethercat_interface
#endif  // ETHERCAT_INTERFACE__EC_PDO_CHANNEL_MANAGER_HPP_
