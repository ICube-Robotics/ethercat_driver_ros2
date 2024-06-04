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

#include <bitset>
#include <iostream>
#include "ethercat_interface/ec_pdo_single_interface_channel_manager.hpp"

namespace ethercat_interface
{

#ifndef CLASSM
#define CLASSM EcPdoSingleInterfaceChannelManager
#else
#error alias CLASSM all ready defined!
#endif  // < CLASSM


CLASSM::EcPdoSingleInterfaceChannelManager()
  : state_interface_index_(std::numeric_limits<size_t>::max()),
  command_interface_index_(std::numeric_limits<size_t>::max()),
  read_function_(nullptr),
  write_function_(nullptr),
  state_interface_name_idx_(0),
  command_interface_name_idx_(0)
{
}

CLASSM::~EcPdoSingleInterfaceChannelManager()
{
}

std::pair<bool, size_t> CLASSM::is_interface_managed(std::string name) const
{
  if (has_command_interface_name()) {
    bool managed = (name == all_command_interface_names[command_interface_name_idx_]);
    if (managed) {
      return std::make_pair(true, 0);
    }
  }
  if (has_state_interface_name()) {
    bool managed = (name == all_state_interface_names[state_interface_name_idx_]);
    if (managed) {
      return std::make_pair(true, 0);
    } else {
      return std::make_pair(false, std::numeric_limits<size_t>::max());
    }
  }
  return std::make_pair(false, std::numeric_limits<size_t>::max());
}

bool CLASSM::load_from_config(YAML::Node channel_config)
{
  // TODO(@yguel@unistra): Use ROS2 logging
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
    std::cerr << "channel: " << index << " : missing channel info" << std::endl;
  }

  // data type
  std::string data_type;
  if (channel_config["type"]) {
    data_type = channel_config["type"].as<std::string>();
    data_type_idx_ = typeIdx(data_type);
    if (0 == data_type_idx_) {
      std::cerr << "channel: " << index << " : unknown data type " << data_type << std::endl;
      return false;
    }
    bits_ = type2bits(data_type);
    read_function_ = ec_pdo_single_read_functions[data_type_idx_];
    write_function_ = ec_pdo_single_write_functions[data_type_idx_];
  } else {
    std::cerr << "channel: " << index << " : missing channel data type info" << std::endl;
  }

  if (channel_config["command_interface"]) {
    auto command_interface_name = channel_config["command_interface"].as<std::string>();
    command_interface_name_idx_ = all_command_interface_names.size();
    all_command_interface_names.push_back(command_interface_name);
    // default value
    if (channel_config["default"]) {
      default_value = channel_config["default"].as<double>();
    }
  }

  if (channel_config["state_interface"]) {
    auto state_interface_name = channel_config["state_interface"].as<std::string>();
    state_interface_name_idx_ = all_state_interface_names.size();
    all_state_interface_names.push_back(state_interface_name);
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
    mask = channel_config["mask"].as<uint8_t>();
    if (!check_type(data_type, mask)) {
      std::cerr << "channel: " << index << " : mask " << std::bitset<8>(mask) <<
        " is not compatible with data type " << data_type << std::endl;
      return false;
    }
  }

  // skip
  if (channel_config["skip"]) {
    skip = channel_config["skip"].as<bool>();
  }

  return true;
}

double CLASSM::ec_read(uint8_t * domain_address, size_t /*i*/)
{
  last_value = read_function_(domain_address, mask);
  last_value = factor * last_value + offset;
  if (is_state_interface_defined() ) {
    state_interface_ptr_->at(state_interface_index_) = last_value;
  }
  return last_value;
}

void CLASSM::ec_read_to_interface(uint8_t * domain_address)
{
  ec_read(domain_address);
  if (is_state_interface_defined() ) {
    state_interface_ptr_->at(state_interface_index_) = last_value;
  }
}

void CLASSM::ec_write(uint8_t * domain_address, double value, size_t /*i*/)
{
  if (RPDO != pdo_type || !allow_ec_write) {
    return;
  }
  if (!std::isnan(value) && !override_command) {
    last_value = factor * value + offset;
    write_function_(domain_address, last_value, mask);
  } else {
    if (!std::isnan(default_value)) {
      last_value = default_value;
      write_function_(domain_address, last_value, mask);
    } else {  // Do nothing
      return;
    }
  }
}

void CLASSM::ec_write_from_interface(uint8_t * domain_address)
{
  if (is_command_interface_defined() ) {
    const auto value = command_interface_ptr_->at(command_interface_index_);
    ec_write(domain_address, value);
  } else {
    if ( (RPDO == pdo_type) && allow_ec_write && !std::isnan(default_value)) {
      last_value = default_value;
      write_function_(domain_address, last_value, mask);
    }
  }
}

#undef CLASSM

}  // < namespace ethercat_interface
