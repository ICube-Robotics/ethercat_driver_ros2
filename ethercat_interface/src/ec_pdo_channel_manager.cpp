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

#include <algorithm>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
// #include <bitset> // For debugging purpose
#include "ethercat_interface/ec_pdo_channel_manager.hpp"

namespace ethercat_interface
{

const std::vector<std::string> ec_pdo_channel_data_types = {
  "unknown",
  "bit",
  "bool",
  "int8", "uint8",
  "int16", "uint16",
  "int32", "uint32",
  "int64", "uint64",
  "float", "real32",
  "double", "real64"
};

const std::vector<uint8_t> ec_pdo_channel_data_bits = {
  0,
  0,
  1,
  8, 8,
  16, 16,
  32, 32,
  64, 64,
  32, 32,
  64, 64
};

std::vector<std::string> all_state_interface_names = {
  "unknown"
};

std::vector<std::string> all_command_interface_names = {
  "unknown"
};

size_t typeIdx(const std::string & type)
{
  // Handle the types of the form bitXXX
  if (type.find("bit") != std::string::npos) {
    return 1;
  }
  // Handle the standard types
  auto it =
    std::find(ec_pdo_channel_data_types.begin(), ec_pdo_channel_data_types.end(), type);
  if (it != ec_pdo_channel_data_types.end()) {
    return std::distance(ec_pdo_channel_data_types.begin(), it);
  }
  // Handle an unknown type
  return 0;
}

uint8_t type2bits(const std::string & type)
{
  size_t type_idx = typeIdx(type);
  // Handle the types of the form bitXXX
  if (1 == type_idx) {
    try {
      std::string n_bits = type.substr(type.find("bit") + 3);
      return static_cast<uint8_t>(std::stoi(n_bits));
    } catch (std::invalid_argument & e) {
      return 0;
    }
  }
  // Handle the other types
  return ec_pdo_channel_data_bits[type_idx];
}

std::string id_and_bits_to_type(size_t type_idx, uint8_t bits)
{
  if (type_idx < ec_pdo_channel_data_types.size()) {
    if (1 == type_idx) {
      return "bit" + std::to_string(bits);
    }
    return ec_pdo_channel_data_types[type_idx];
  } else {
    throw std::out_of_range(
            "id_and_bits_to_type: unknown index type (type_idx must be < " +
            std::to_string(
              ec_pdo_channel_data_types.size()) + ", the size of known types, instead of " +
            std::to_string(type_idx) + " )");
  }
}

#ifndef CLASSM
#define CLASSM EcPdoChannelManager
#else
#error alias CLASSM all ready defined!
#endif  // < CLASSM

CLASSM::EcPdoChannelManager() {
}

CLASSM::~EcPdoChannelManager() {
}

ec_pdo_entry_info_t CLASSM::get_pdo_entry_info()
{
  RCLCPP_INFO(
    rclcpp::get_logger("EcPdoChannelManager"),
    "{0x%x, 0x%x, %d}",
    index,
    static_cast<uint16_t>(sub_index),
    static_cast<int>(pdo_bits())
  );

  return {index, sub_index, pdo_bits()};
}

void CLASSM::setup_interface_ptrs(
  std::vector<double> * state_interface,
  std::vector<double> * command_interface)
{
  command_interface_ptr_ = command_interface;
  state_interface_ptr_ = state_interface;
}

void CLASSM::ec_update(uint8_t * domain_address)
{
  ec_read_to_interface(domain_address);
  ec_write_from_interface(domain_address);
}


#undef CLASSM

double uint8_read(uint8_t * domain_address, uint8_t /*data_mask*/)
{
  return static_cast<double>(EC_READ_U8(domain_address));
}

double int8_read(uint8_t * domain_address, uint8_t /*data_mask*/)
{
  return static_cast<double>(EC_READ_S8(domain_address));
}

double uint16_read(uint8_t * domain_address, uint8_t /*data_mask*/)
{
  return static_cast<double>(EC_READ_U16(domain_address));
}

double int16_read(uint8_t * domain_address, uint8_t /*data_mask*/)
{
  return static_cast<double>(EC_READ_S16(domain_address));
}

double uint32_read(uint8_t * domain_address, uint8_t /*data_mask*/)
{
  return static_cast<double>(EC_READ_U32(domain_address));
}

double int32_read(uint8_t * domain_address, uint8_t /*data_mask*/)
{
  return static_cast<double>(EC_READ_S32(domain_address));
}

double uint64_read(uint8_t * domain_address, uint8_t /*data_mask*/)
{
  return static_cast<double>(EC_READ_U64(domain_address));
}

double int64_read(uint8_t * domain_address, uint8_t /*data_mask*/)
{
  return static_cast<double>(EC_READ_S64(domain_address));
}

double real32_read(uint8_t * domain_address, uint8_t /*data_mask*/)
{
  uint32_t raw = EC_READ_U32(domain_address);
  float value;
  std::memcpy(&value, &raw, sizeof(value));
  return value;
}

double real64_read(uint8_t * domain_address, uint8_t /*data_mask*/)
{
  uint64_t raw = EC_READ_U64(domain_address);
  double value;
  std::memcpy(&value, &raw, sizeof(value));
  return value;
}

double bool_read(uint8_t * domain_address, uint8_t data_mask)
{
  return (EC_READ_U8(domain_address) & data_mask) ? 1. : 0.;
}

double octet_read(uint8_t * domain_address, uint8_t data_mask)
{
  return static_cast<double>(EC_READ_U8(domain_address) & data_mask);
}

const SingleReadFunctionType ec_pdo_single_read_functions[] = {
  NULL,
  octet_read,
  bool_read,
  int8_read, uint8_read,
  int16_read, uint16_read,
  int32_read, uint32_read,
  int64_read, uint64_read,
  real32_read, real32_read,
  real64_read, real64_read,
  octet_read
};

/*############################*/
/* Single Write functions     */
/*############################*/

void uint8_write(uint8_t * domain_address, double value, uint8_t /*data_mask*/)
{
  EC_WRITE_U8(domain_address, static_cast<uint8_t>(value));
}

void int8_write(uint8_t * domain_address, double value, uint8_t /*data_mask*/)
{
  EC_WRITE_S8(domain_address, static_cast<int8_t>(value));
}

void uint16_write(uint8_t * domain_address, double value, uint8_t /*data_mask*/)
{
  EC_WRITE_U16(domain_address, static_cast<uint16_t>(value));
}

void int16_write(uint8_t * domain_address, double value, uint8_t /*data_mask*/)
{
  EC_WRITE_S16(domain_address, static_cast<int16_t>(value));
}

void uint32_write(uint8_t * domain_address, double value, uint8_t /*data_mask*/)
{
  EC_WRITE_U32(domain_address, static_cast<uint32_t>(value));
}

void int32_write(uint8_t * domain_address, double value, uint8_t /*data_mask*/)
{
  EC_WRITE_S32(domain_address, static_cast<int32_t>(value));
}

void uint64_write(uint8_t * domain_address, double value, uint8_t /*data_mask*/)
{
  EC_WRITE_U64(domain_address, static_cast<uint64_t>(value));
}

void int64_write(uint8_t * domain_address, double value, uint8_t /*data_mask*/)
{
  EC_WRITE_S64(domain_address, static_cast<int64_t>(value));
}

void real32_write(uint8_t * domain_address, double value, uint8_t /*data_mask*/)
{
  float f = static_cast<float>(value);
  uint32_t raw;
  std::memcpy(&raw, &f, sizeof(raw));
  EC_WRITE_U32(domain_address, static_cast<uint32_t>(raw));
}

void real64_write(uint8_t * domain_address, double value, uint8_t /*data_mask*/)
{
  uint64_t raw;
  std::memcpy(&raw, &value, sizeof(raw));
  EC_WRITE_U64(domain_address, static_cast<uint64_t>(raw));
}

/** @brief Helper function that counts the number of bits in an octet */
inline
uint8_t count_bits(uint8_t octet)
{
  return __builtin_popcount(octet);
}

bool check_type(const std::string & type, uint8_t mask)
{
  if ("bool" == type) {
    return 1 == count_bits(mask);
  }
  return true;
}

/** @brief Modify one bit defined by the mask
 * \pre The mask must contain only one bit set to one
 */
void bool_compose(uint8_t * domain_address, double value, uint8_t data_mask)
{
  uint8_t buffer = EC_READ_U8(domain_address);
  // Clear the bit
  buffer &= ~(data_mask);
  if (value) {  // Set the bit
    buffer |= data_mask;
  }
  EC_WRITE_U8(domain_address, buffer);
}

/** Modify only the bits set to one in the mask */
void octet_compose(uint8_t * domain_address, double value, uint8_t data_mask)
{
  uint8_t buffer = EC_READ_U8(domain_address);
  // Clear the bits
  buffer &= ~(data_mask);
  uint8_t compose_buffer = static_cast<uint8_t>(value) & data_mask;
  EC_WRITE_U8(domain_address, buffer | compose_buffer);
}

/** Modify the whole octet with the result of the applied mask */
void octet_override(uint8_t * domain_address, double value, uint8_t data_mask)
{
  EC_WRITE_U8(domain_address, static_cast<uint8_t>(value) & data_mask);
}

const SingleWriteFunctionType ec_pdo_single_write_functions[] = {
  NULL,
  octet_compose,
  bool_compose,
  int8_write, uint8_write,
  int16_write, uint16_write,
  int32_write, uint32_write,
  int64_write, uint64_write,
  real32_write, real32_write,
  real64_write, real64_write,
  octet_override
};

/*###############################*/
/* End of Single Write functions */
/*###############################*/

}  // < namespace ethercat_interface
