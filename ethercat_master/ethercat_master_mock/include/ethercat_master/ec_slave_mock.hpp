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

#include "ethercat_interface/ec_sdo_manager.hpp"
#include "ethercat_interface/ec_slave.hpp"

namespace ethercat_master
{

class MockSlave
{
public:
  explicit MockSlave(ethercat_interface::EcSlave * slave);
  ~MockSlave();
  /** read or write data to the domain */
  int process_data(size_t index, uint8_t * domain_address);
  bool initialized();
  void set_state_is_operational(bool value);
  /** Assign activate DC synchronization. return activate word*/
  int dc_sync();

  bool setupSlave(
    std::unordered_map<std::string, std::string> slave_paramters,
    std::vector<double> * state_interface,
    std::vector<double> * command_interface);

  uint32_t vendor_id;
  uint32_t product_id;
  int bus_position;
  int bus_alias;

  std::vector<ethercat_interface::SdoConfigEntry> sdo_config;

protected:
  ethercat_interface::EcSlave * slave_;
  std::vector<double> * state_interface_ptr_;
  std::vector<double> * command_interface_ptr_;
  std::unordered_map<std::string, std::string> paramters_;
  bool is_operational_ = false;
};
}  // namespace ethercat_master
#endif  // ETHERCAT_MASTER__EC_SLAVE_MOCK_HPP_
