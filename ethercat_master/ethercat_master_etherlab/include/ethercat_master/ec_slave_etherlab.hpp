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
#include "ethercat_interface/ec_slave_base.hpp"

namespace ethercat_master
{

class EtherlabSlave
{
public:
  explicit EtherlabSlave(std::shared_ptr<ethercat_interface::EcSlaveBase> slave);
  ~EtherlabSlave();

  /** a pointer to syncs. return &syncs[0] */
  const ec_sync_info_t * syncs();
  bool initialized();
  /** Assign activate DC synchronization. return activate word*/
  int assign_activate_dc_sync();

  /** number of elements in the syncs array. */
  size_t sync_size();
  /** a pointer to all PDO entries */
  const ec_pdo_entry_info_t * channels();
  /** a map from domain index to pdo indices in that domain.
  *  map<domain index, vector<channels_ indices> > */
  typedef std::map<unsigned int, std::vector<unsigned int>> DomainMap;
  void domains(DomainMap & /*domains*/) const;

  std::shared_ptr<ethercat_interface::EcSlaveBase> get_slave() {return slave_;}

protected:
  std::shared_ptr<ethercat_interface::EcSlaveBase> slave_;

  std::vector<ec_pdo_info_t> rpdos_;
  std::vector<ec_pdo_info_t> tpdos_;
  std::vector<ec_pdo_entry_info_t> all_channels_;
  std::vector<bool> all_channels_skip_list_;
  std::vector<ec_sync_info_t> syncs_;
  std::vector<unsigned int> domain_map_;

  bool setup_slave();
  void setup_syncs();
  ec_direction_t set_sm_type(int type);
  ec_watchdog_mode_t set_sm_watchdog(int watchdog);
};
}  // namespace ethercat_master
#endif  // ETHERCAT_MASTER__EC_SLAVE_ETHERLAB_HPP_
