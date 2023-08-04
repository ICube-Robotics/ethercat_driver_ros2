// Copyright 2022 ICUBE Laboratory, University of Strasbourg
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

#ifndef ETHERCAT_INTERFACE__EC_SLAVE_HPP_
#define ETHERCAT_INTERFACE__EC_SLAVE_HPP_

#include <map>
#include <vector>
#include <unordered_map>
#include <iostream>
#include <cmath>
#include <string>

#include "ethercat_interface/ec_sdo_manager.hpp"
#include "ethercat_interface/ec_pdo_channel_manager.hpp"
#include "ethercat_interface/ec_sync_manager.hpp"
#include "ethercat_interface/ec_buffer_tools.h"

namespace ethercat_interface
{

typedef std::unordered_map<uint16_t,
    std::vector<ethercat_interface::EcPdoChannelManager>> pdoc_map_t;
typedef std::vector<ethercat_interface::EcPdoChannelManager> pdoc_vec_t;
typedef std::vector<ethercat_interface::SMConfig> smc_vec_t;
typedef std::vector<ethercat_interface::SdoConfigEntry> sdoc_vec_t;

class EcSlave
{
public:
  EcSlave() {}
  ~EcSlave() {}
  /** read or write data to the domain */
  virtual int process_data(size_t /*index*/, uint8_t * /*domain_address*/) {return 0;}
  /** Assign activate DC synchronization. return activate word*/
  virtual int dc_sync() {return assign_activate_;}
  bool initialized() {return is_initialized_;}
  virtual bool setup_slave(
    std::unordered_map<std::string, std::string> slave_paramters,
    std::vector<double> * state_interface,
    std::vector<double> * command_interface)
  {
    state_interface_ptr_ = state_interface;
    command_interface_ptr_ = command_interface;
    paramters_ = slave_paramters;
    is_initialized_ = true;
    return true;
  }

  uint32_t get_vendor_id() {return vendor_id_;}
  uint32_t get_product_id() {return product_id_;}

  pdoc_map_t get_pdo_config_map()
  {
    return pdo_config_map_;
  }
  pdoc_vec_t get_pdo_channel_config()
  {
    return pdo_channels_info_;
  }
  smc_vec_t get_sm_config()
  {
    return sm_config_;
  }
  sdoc_vec_t get_sdo_config()
  {
    return sdo_config_;
  }

protected:
  std::vector<double> * state_interface_ptr_;
  std::vector<double> * command_interface_ptr_;
  std::unordered_map<std::string, std::string> paramters_;
  bool is_initialized_ = false;
  uint32_t vendor_id_ = 0;
  uint32_t product_id_ = 0;
  uint32_t assign_activate_ = 0x00;

  pdoc_map_t pdo_config_map_;
  pdoc_vec_t pdo_channels_info_;
  smc_vec_t sm_config_;
  sdoc_vec_t sdo_config_;
};

}  // namespace ethercat_interface
#endif  // ETHERCAT_INTERFACE__EC_SLAVE_HPP_
