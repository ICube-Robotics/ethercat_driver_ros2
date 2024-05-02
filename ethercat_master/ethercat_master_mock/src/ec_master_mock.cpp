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

#include "ethercat_master/ec_master_mock.hpp"

namespace ethercat_master
{

MockMaster::MockMaster()
{
  interval_ = 0;
}

MockMaster::~MockMaster()
{
}

bool MockMaster::init(std::string master_interface)
{
  return true;
}

bool MockMaster::add_slave(std::shared_ptr<ethercat_interface::EcSlave> slave)
{
  // configure slave in master
  slave_list_.emplace_back();
  slave_list_.back() = std::make_shared<MockSlave>(slave);
  return true;
}

bool MockMaster::configure_slaves()
{
  return true;
}

bool MockMaster::start()
{
  return true;
}

bool MockMaster::stop()
{
  return true;
}

bool MockMaster::spin_slaves_until_operational()
{
  return true;
}

bool MockMaster::read_process_data()
{
  return true;
}

bool MockMaster::write_process_data()
{
  return true;
}
}  // namespace ethercat_master

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ethercat_master::MockMaster, ethercat_interface::EcMaster)
