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

#include "ethercat_master/ec_slave_etherlab.hpp"

namespace ethercat_master
{
EtherlabSlave::EtherlabSlave(ethercat_interface::EcSlave * slave)
{
  slave_ = slave;
}

EtherlabSlave::~EtherlabSlave()
{
}

int EtherlabSlave::process_data(
  size_t pdo_mapping_index, size_t pdo_channel_index, uint8_t * domain_address)
{
  slave_->process_data(pdo_mapping_index, pdo_channel_index, domain_address);
  return 0;
}

const ec_sync_info_t * EtherlabSlave::syncs()
{
  return syncs_.data();
}

bool EtherlabSlave::initialized()
{
  return slave_->initialized();
}

void EtherlabSlave::set_state_is_operational(bool value)
{
  is_operational_ = value;
}

int EtherlabSlave::dc_sync()
{
  return slave_->dc_sync();
}

size_t EtherlabSlave::sync_size()
{
  return syncs_.size();
}

const ec_pdo_entry_info_t * EtherlabSlave::channels()
{
  return all_channels_.data();
}

void EtherlabSlave::domains(DomainMap & domains) const
{
  domains = {{0, domain_map_}};
}

ec_direction_t EtherlabSlave::set_sm_type(int type)
{
  return (type == 1) ? EC_DIR_INPUT : EC_DIR_OUTPUT;
}

ec_watchdog_mode_t EtherlabSlave::set_sm_watchdog(int watchdog)
{
  switch (watchdog) {
    case -1:
      return EC_WD_DISABLE;
    case 0:
      return EC_WD_DEFAULT;
    case 1:
      return EC_WD_ENABLE;
  }
}

void EtherlabSlave::setup_syncs()
{
  if (slave_->get_sm_config().size() == 0) {
    syncs_.push_back({0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE});
    syncs_.push_back({1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE});
    syncs_.push_back({2, EC_DIR_OUTPUT, rpdos_.size(), rpdos_.data(), EC_WD_ENABLE});
    syncs_.push_back({3, EC_DIR_INPUT, tpdos_.size(), tpdos_.data(), EC_WD_DISABLE});
  } else {
    for (auto & sm : slave_->get_sm_config()) {
      if (sm.pdo_name == "null") {
        syncs_.push_back({sm.index, set_sm_type(sm.type), 0, NULL, set_sm_watchdog(sm.watchdog)});
      } else if (sm.pdo_name == "rpdo") {
        syncs_.push_back(
          {sm.index, set_sm_type(sm.type), rpdos_.size(),
            rpdos_.data(), set_sm_watchdog(sm.watchdog)});
      } else if (sm.pdo_name == "tpdo") {
        syncs_.push_back(
          {sm.index, set_sm_type(sm.type), tpdos_.size(),
            tpdos_.data(), set_sm_watchdog(sm.watchdog)});
      }
    }
  }
  syncs_.push_back({0xff});
}

bool EtherlabSlave::setup_slave(
  std::unordered_map<std::string, std::string> slave_paramters,
  std::vector<double> * state_interface,
  std::vector<double> * command_interface)
{
  state_interface_ptr_ = state_interface;
  command_interface_ptr_ = command_interface;
  paramters_ = slave_paramters;

  if (!slave_->setup_slave(slave_paramters, state_interface, command_interface)) {
    return false;
  }

  auto channels_nbr = 0;

  for (auto & mapping : slave_->get_pdo_config()) {
    channels_nbr += mapping.pdo_channel_config.size();
  }

  all_channels_.reserve(channels_nbr);

  channels_nbr = 0;

  for (auto & mapping : slave_->get_pdo_config()) {
    for (auto & channel_info : mapping.pdo_channel_config) {
      all_channels_.push_back(
        {
          channel_info.index,
          channel_info.sub_index,
          channel_info.get_bits_size()
        });
    }
    if (mapping.pdo_type == ethercat_interface::RPDO) {
      rpdos_.push_back(
        {
          mapping.index,
          mapping.pdo_channel_config.size(),
          all_channels_.data() + channels_nbr
        }
      );
      channels_nbr += mapping.pdo_channel_config.size();
    } else if (mapping.pdo_type == ethercat_interface::TPDO) {
      tpdos_.push_back(
        {
          mapping.index,
          mapping.pdo_channel_config.size(),
          all_channels_.data() + channels_nbr
        }
      );
      channels_nbr += mapping.pdo_channel_config.size();
    }
  }

  // Remove gaps from domain mapping
  for (auto i = 0ul; i < all_channels_.size(); i++) {
    if (all_channels_[i].index != 0x0000) {
      domain_map_.push_back(i);
    }
  }

  if (slave_paramters.find("position") != slave_paramters.end()) {
    bus_position_ = std::stoi(slave_paramters["position"]);
  } else {
    bus_position_ = 0;
  }

  if (slave_paramters.find("alias") != slave_paramters.end()) {
    bus_alias_ = std::stoi(slave_paramters["alias"]);
  } else {
    bus_alias_ = 0;
  }

  setup_syncs();

  return true;
}

uint32_t EtherlabSlave::get_vendor_id()
{
  return slave_->get_vendor_id();
}

uint32_t EtherlabSlave::get_product_id()
{
  return slave_->get_product_id();
}

int EtherlabSlave::get_bus_position()
{
  return bus_position_;
}

int EtherlabSlave::get_bus_alias()
{
  return bus_alias_;
}

ethercat_interface::pdo_config_t EtherlabSlave::get_pdo_config()
{
  return slave_->get_pdo_config();
}

ethercat_interface::sm_config_t EtherlabSlave::get_sm_config()
{
  return slave_->get_sm_config();
}

ethercat_interface::sdo_config_t EtherlabSlave::get_sdo_config()
{
  return slave_->get_sdo_config();
}
}  // namespace ethercat_master
