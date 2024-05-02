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

#ifndef ETHERCAT_INTERFACE__EC_PDO_CHANNEL_MANAGER_HPP_
#define ETHERCAT_INTERFACE__EC_PDO_CHANNEL_MANAGER_HPP_

#include <string>
#include <vector>
#include <limits>

#include "yaml-cpp/yaml.h"
#include "ethercat_interface/ec_buffer_tools.h"

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
  EcPdoChannelManager(
    uint16_t index,
    uint8_t sub_index,
    PdoType pdo_type,
    std::string data_type,
    std::string interface_name = "",
    uint8_t data_mask = 255,
    double default_value = std::numeric_limits<double>::quiet_NaN(),
    double factor = 1,
    double offset = 0)
  : index(index),
    sub_index(sub_index),
    pdo_type(pdo_type),
    data_type(data_type),
    interface_name(interface_name),
    data_mask(data_mask),
    default_value(default_value),
    factor(factor),
    offset(offset)
  {}
  ~EcPdoChannelManager() {}
  void setup_interface_ptrs(
    std::vector<double> * state_interface,
    std::vector<double> * command_interface)
  {
    command_interface_ptr_ = command_interface;
    state_interface_ptr_ = state_interface;
  }

  double ec_read(uint8_t * domain_address)
  {
    if (data_type == "uint8") {
      last_value = static_cast<double>(read_u8(domain_address));
    } else if (data_type == "int8") {
      last_value = static_cast<double>(read_s8(domain_address));
    } else if (data_type == "uint16") {
      last_value = static_cast<double>(read_u16(domain_address));
    } else if (data_type == "int16") {
      last_value = static_cast<double>(read_s16(domain_address));
    } else if (data_type == "uint32") {
      last_value = static_cast<double>(read_u32(domain_address));
    } else if (data_type == "int32") {
      last_value = static_cast<double>(read_s32(domain_address));
    } else if (data_type == "uint64") {
      last_value = static_cast<double>(read_u64(domain_address));
    } else if (data_type == "int64") {
      last_value = static_cast<double>(read_s64(domain_address));
    } else if (data_type == "bool") {
      last_value = (read_u8(domain_address) & data_mask) ? 1 : 0;
    } else {
      last_value = static_cast<double>(read_u8(domain_address) & data_mask);
    }
    last_value = factor * last_value + offset;
    return last_value;
  }

  void ec_write(uint8_t * domain_address, double value)
  {
    if (data_type == "uint8") {
      write_u8(domain_address, static_cast<uint8_t>(value));
    } else if (data_type == "int8") {
      write_s8(domain_address, static_cast<int8_t>(value));
    } else if (data_type == "uint16") {
      write_u16(domain_address, static_cast<uint16_t>(value));
    } else if (data_type == "int16") {
      write_s16(domain_address, static_cast<int16_t>(value));
    } else if (data_type == "uint32") {
      write_u32(domain_address, static_cast<uint32_t>(value));
    } else if (data_type == "int32") {
      write_s32(domain_address, static_cast<int32_t>(value));
    } else if (data_type == "uint64") {
      write_u64(domain_address, static_cast<uint64_t>(value));
    } else if (data_type == "int64") {
      write_s64(domain_address, static_cast<int64_t>(value));
    } else {
      buffer_ = read_u8(domain_address);
      if (popcount(data_mask) == 1) {
        buffer_ &= ~(data_mask);
        if (value) {buffer_ |= data_mask;}
      } else if (data_mask != 0) {
        buffer_ = 0;
        buffer_ |= (static_cast<uint8_t>(value) & data_mask);
      }
      write_u8(domain_address, buffer_);
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
      if (interface_index >= 0 &&
        !std::isnan(command_interface_ptr_->at(interface_index)) &&
        !override_command)
      {
        ec_write(domain_address, factor * command_interface_ptr_->at(interface_index) + offset);
      } else {
        if (!std::isnan(default_value)) {
          ec_write(domain_address, default_value);
        }
      }
    }
  }

  bool load_from_config(YAML::Node channel_config)
  {
    // index
    if (channel_config["index"]) {
      index = channel_config["index"].as<uint16_t>();
    } else {
      std::cerr << "missing channel index info" << std::endl;
    }
    // sub_index
    if (channel_config["sub_index"]) {
      sub_index = channel_config["sub_index"].as<uint8_t>();
    } else {
      std::cerr << "channel " << index << ": missing channel info" << std::endl;
    }
    // data type
    if (channel_config["type"]) {
      data_type = channel_config["type"].as<std::string>();
    } else {
      std::cerr << "channel " << index << ": missing channel data type info" << std::endl;
    }

    if (pdo_type == RPDO) {
      // interface name
      if (channel_config["command_interface"]) {
        interface_name = channel_config["command_interface"].as<std::string>();
      }
      // default value
      if (channel_config["default"]) {
        default_value = channel_config["default"].as<double>();
      }

    } else if (pdo_type == TPDO) {
      // interface name
      if (channel_config["state_interface"]) {
        interface_name = channel_config["state_interface"].as<std::string>();
      }
    }

    // factor
    if (channel_config["factor"]) {
      factor = channel_config["factor"].as<double>();
    }
    // offset
    if (channel_config["offset"]) {
      offset = channel_config["offset"].as<double>();
    }
    // mask
    if (channel_config["mask"]) {
      data_mask = channel_config["mask"].as<uint8_t>();
    }

    return true;
  }

  uint8_t type2bits(std::string type)
  {
    if (type == "bool") {
      return 1;
    } else if (type == "int16" || type == "uint16") {
      return 16;
    } else if (type == "int8" || type == "uint8") {
      return 8;
    } else if (type == "int16" || type == "uint16") {
      return 16;
    } else if (type == "int32" || type == "uint32") {
      return 32;
    } else if (type == "int64" || type == "uint64") {
      return 64;
    } else if (type.find("bit") != std::string::npos) {
      std::string n_bits = type.substr(type.find("bit") + 3);
      return static_cast<uint8_t>(std::stoi(n_bits));
    }
    return -1;
  }

  uint8_t get_bits_size() {return type2bits(data_type);}

  uint16_t index;
  uint8_t sub_index;
  PdoType pdo_type;
  std::string data_type;
  std::string interface_name;
  uint8_t data_mask = 255;
  double default_value = std::numeric_limits<double>::quiet_NaN();
  int interface_index = -1;
  double last_value = std::numeric_limits<double>::quiet_NaN();
  bool allow_ec_write = true;
  bool override_command = false;
  double factor = 1;
  double offset = 0;

private:
  std::vector<double> * command_interface_ptr_;
  std::vector<double> * state_interface_ptr_;
  uint8_t buffer_ = 0;

  int popcount(uint8_t x)
  {
    int count = 0;
    for (; x != 0; x >>= 1) {if (x & 1) {count++;}}
    return count;
  }
};

}  // namespace ethercat_interface
#endif  // ETHERCAT_INTERFACE__EC_PDO_CHANNEL_MANAGER_HPP_
