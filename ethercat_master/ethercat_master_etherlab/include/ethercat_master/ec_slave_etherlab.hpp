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

#ifndef ETHERCAT_MASTER__EC_SLAVE_ETHERLAB_HPP_
#define ETHERCAT_MASTER__EC_SLAVE_ETHERLAB_HPP_

#include <ecrt.h>
#include <map>
#include <vector>
#include <unordered_map>
#include <iostream>
#include <cmath>
#include <string>
#include <memory>

#include "ethercat_interface/ec_sdo_manager.hpp"
#include "ethercat_interface/ec_slave.hpp"

namespace ethercat_master
{

class EtherlabSlave
{
public:
  explicit EtherlabSlave(std::shared_ptr<ethercat_interface::EcSlave> slave);
  ~EtherlabSlave();
  /** read or write data to the domain */
  int process_data(size_t pdo_mapping_index, size_t pdo_channel_index, uint8_t * domain_address);
  /** a pointer to syncs. return &syncs[0] */
  const ec_sync_info_t * syncs();
  bool initialized();
  void set_state_is_operational(bool value);
  /** Assign activate DC synchronization. return activate word*/
  int dc_sync();
  /** number of elements in the syncs array. */
  size_t sync_size();
  /** a pointer to all PDO entries */
  const ec_pdo_entry_info_t * channels();
  /** a map from domain index to pdo indices in that domain.
  *  map<domain index, vector<channels_ indices> > */
  typedef std::map<unsigned int, std::vector<unsigned int>> DomainMap;
  void domains(DomainMap & /*domains*/) const;

  uint32_t get_vendor_id();
  uint32_t get_product_id();
  int get_bus_position();
  int get_bus_alias();

  ethercat_interface::pdo_config_t get_pdo_config();
  ethercat_interface::sm_config_t get_sm_config();
  ethercat_interface::sdo_config_t get_sdo_config();

protected:
  std::shared_ptr<ethercat_interface::EcSlave> slave_;
  bool is_operational_ = false;
  int bus_position_;
  int bus_alias_;

  std::vector<ec_pdo_info_t> rpdos_;
  std::vector<ec_pdo_info_t> tpdos_;
  std::vector<ec_pdo_entry_info_t> all_channels_;
  std::vector<ec_sync_info_t> syncs_;
  std::vector<unsigned int> domain_map_;

  bool setup_slave();
  void setup_syncs();
  ec_direction_t set_sm_type(int type);
  ec_watchdog_mode_t set_sm_watchdog(int watchdog);
};
}  // namespace ethercat_master
#endif  // ETHERCAT_MASTER__EC_SLAVE_ETHERLAB_HPP_
