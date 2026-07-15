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
#include "rclcpp/rclcpp.hpp"
#include "ethercat_master/ec_slave_etherlab.hpp"

namespace ethercat_master
{
EtherlabSlave::EtherlabSlave(std::shared_ptr<ethercat_interface::EcSlaveBase> slave)
{
  slave_ = slave;
  setup_slave();
}
int EtherlabSlave::assign_activate_dc_sync()
{
  return slave_->assign_activate_dc_sync();
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
    default:
      return EC_WD_DEFAULT;
  }
}

void EtherlabSlave::setup_syncs()
{
  if (slave_->get_sm_config().size() == 0) {
    syncs_.push_back({0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE});
    syncs_.push_back({1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE});
    syncs_.push_back(
      {2, EC_DIR_OUTPUT, (unsigned int)(rpdos_.size()), rpdos_.data(),
        EC_WD_ENABLE});
    syncs_.push_back(
      {3, EC_DIR_INPUT, (unsigned int)(tpdos_.size()), tpdos_.data(),
        EC_WD_DISABLE});
  } else {
    for (auto & sm : slave_->get_sm_config()) {
      if (sm.pdo_name == "null") {
        syncs_.push_back({sm.index, set_sm_type(sm.type), 0, NULL, set_sm_watchdog(sm.watchdog)});
      } else if (sm.pdo_name == "rpdo") {
        syncs_.push_back(
          {sm.index, set_sm_type(sm.type), (unsigned int)(rpdos_.size()),
            rpdos_.data(), set_sm_watchdog(sm.watchdog)});
      } else if (sm.pdo_name == "tpdo") {
        syncs_.push_back(
          {sm.index, set_sm_type(sm.type), (unsigned int)(tpdos_.size()),
            tpdos_.data(), set_sm_watchdog(sm.watchdog)});
      }
    }
  }
  syncs_.push_back({0xff, EC_DIR_INVALID, 0, nullptr, EC_WD_DISABLE});
}

bool EtherlabSlave::setup_slave()
{
  auto channels_nbr = slave_->get_pdo_channels_info().size();

  all_channels_.reserve(channels_nbr);
  all_channels_skip_list_.reserve(channels_nbr);
  channels_nbr = 0;
  for (auto & channel_info : slave_->get_pdo_channels_info()) {
    ec_pdo_entry_info_t channel_entry_info = {channel_info->index, channel_info->sub_index,
      channel_info->pdo_bits()};
    all_channels_.push_back(channel_entry_info);
    all_channels_skip_list_.push_back(channel_info->skip);
  }

  for (auto & pdo_info : slave_->get_pdo_info()) {
    if (pdo_info.pdo_type == ethercat_interface::RPDO) {
      rpdos_.push_back(
        {pdo_info.index,
          pdo_info.n_entries,
          all_channels_.data() + channels_nbr});
      channels_nbr += pdo_info.n_entries;
    } else if (pdo_info.pdo_type == ethercat_interface::TPDO) {
      tpdos_.push_back(
        {pdo_info.index,
          pdo_info.n_entries,
          all_channels_.data() + channels_nbr});
      channels_nbr += pdo_info.n_entries;
    }
  }

    // Remove gaps from domain mapping
  for (auto i = 0ul; i < all_channels_.size(); i++) {
    if (all_channels_[i].index != 0x0000 && all_channels_skip_list_[i] != true) {
      domain_map_.push_back(i);
    }
  }

  setup_syncs();

  return true;
}

EtherlabSlave::~EtherlabSlave()
{
  slave_.reset();
  RCLCPP_INFO(rclcpp::get_logger("EtherlabSlave"), "EtherlabSlave destroyed");
}

size_t EtherlabSlave::sync_size()
{
  return syncs_.size();
}

const ec_sync_info_t * EtherlabSlave::syncs()
{
  return syncs_.data();
}
void EtherlabSlave::domains(DomainMap & domains) const
{
  domains = {{0, domain_map_}};
}

const ec_pdo_entry_info_t * EtherlabSlave::channels()
{
  return all_channels_.data();
}

bool EtherlabSlave::initialized()
{
  return slave_->initialized();
}

  /*uint32_t EtherlabSlave::get_vendor_id()
  {
    return slave_->get_vendor_id();
  }*/

  /*uint32_t EtherlabSlave::get_product_id()
  {
    return slave_->get_product_id();
  }*/

  /*int EtherlabSlave::get_bus_position()
  {
    return slave_->get_position();
  }*/

  /*int EtherlabSlave::get_bus_alias()
  {
    return slave_->get_alias();
  }*/

}  //  namespace ethercat_master
