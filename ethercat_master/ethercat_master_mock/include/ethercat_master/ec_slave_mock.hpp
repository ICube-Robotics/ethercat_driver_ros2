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

#ifndef ETHERCAT_MASTER__EC_SLAVE_MOCK_HPP_
#define ETHERCAT_MASTER__EC_SLAVE_MOCK_HPP_

#include <map>
#include <vector>
#include <unordered_map>
#include <string>
#include <memory>

#include "ethercat_interface/ec_sdo_manager.hpp"
#include "ethercat_interface/ec_slave.hpp"

namespace ethercat_master
{

class MockSlave
{
public:
  explicit MockSlave(std::shared_ptr<ethercat_interface::EcSlave> slave);
  ~MockSlave();
  /** read or write data to the domain */
  int process_data(size_t pdo_mapping_index, size_t pdo_channel_index, uint8_t * domain_address);
  bool initialized();
  void set_state_is_operational(bool value);
  /** Assign activate DC synchronization. return activate word*/
  int dc_sync();

  uint32_t get_vendor_id();
  uint32_t get_product_id();
  int get_bus_position();

  ethercat_interface::pdo_config_t get_pdo_config();
  ethercat_interface::sm_config_t get_sm_config();
  ethercat_interface::sdo_config_t get_sdo_config();

  std::vector<ethercat_interface::SdoConfigEntry> sdo_config;

protected:
  std::shared_ptr<ethercat_interface::EcSlave> slave_;
  int bus_position_;

  bool setup_slave();
};
}  // namespace ethercat_master
#endif  // ETHERCAT_MASTER__EC_SLAVE_MOCK_HPP_
