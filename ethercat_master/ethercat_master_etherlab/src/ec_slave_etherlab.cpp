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
  vendor_id = slave->vendor_id;
  product_id = slave->product_id;
  sdo_config = slave->sdo_config;
}

EtherlabSlave::~EtherlabSlave()
{
}

int EtherlabSlave::process_data(size_t index, uint8_t * domain_address)
{
  slave_->process_data(index, domain_address);
  return 0;
}

const ec_sync_info_t * EtherlabSlave::syncs()
{
  return NULL;
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
  return 0;
}

void EtherlabSlave::domains(DomainMap & /*domains*/) const
{
}

bool EtherlabSlave::setupSlave(
  std::unordered_map<std::string, std::string> slave_paramters,
  std::vector<double> * state_interface,
  std::vector<double> * command_interface)
{
  state_interface_ptr_ = state_interface;
  command_interface_ptr_ = command_interface;
  paramters_ = slave_paramters;
  bus_position = std::stoi(slave_paramters["position"]);
  bus_alias = std::stoi(slave_paramters["alias"]);
  return true;
}
}  // namespace ethercat_master
