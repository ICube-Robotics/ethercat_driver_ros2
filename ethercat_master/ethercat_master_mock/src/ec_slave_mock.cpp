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

#include "ethercat_master/ec_slave_mock.hpp"

namespace ethercat_master
{
MockSlave::MockSlave(std::shared_ptr<ethercat_interface::EcSlave> slave)
{
  slave_ = slave;
  setup_slave();
}

MockSlave::~MockSlave()
{
}

int MockSlave::process_data(
  size_t pdo_mapping_index, size_t pdo_channel_index, uint8_t * domain_address)
{
  slave_->process_data(pdo_mapping_index, pdo_channel_index, domain_address);
  return 0;
}

bool MockSlave::initialized()
{
  return slave_->initialized();
}

void MockSlave::set_state_is_operational(bool value)
{
  slave_->set_state_is_operational(value);
}

int MockSlave::dc_sync()
{
  return slave_->dc_sync();
}

bool MockSlave::setup_slave()
{
  if (slave_->get_slave_parameters().find("position") != slave_->get_slave_parameters().end()) {
    bus_position_ = std::stoi(slave_->get_slave_parameters()["position"]);
  } else {
    bus_position_ = 0;
  }
  return true;
}
}  // namespace ethercat_master
