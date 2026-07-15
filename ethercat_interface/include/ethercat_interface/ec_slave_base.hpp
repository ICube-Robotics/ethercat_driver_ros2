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

#ifndef ETHERCAT_INTERFACE__EC_SLAVE_BASE_HPP_
#define ETHERCAT_INTERFACE__EC_SLAVE_BASE_HPP_

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

#include "rclcpp/rclcpp.hpp"


namespace ethercat_interface
{

//typedef std::vector<ethercat_interface::EcPdoChannelManager> pdo_channels_t;
//typedef std::vector<ethercat_interface::SMConfig> sm_config_t;
//typedef std::vector<ethercat_interface::SdoConfigEntry> sdo_config_t;
/*typedef struct
{
  uint16_t index;
  ethercat_interface::PdoType pdo_type;
  std::vector<ethercat_interface::EcPdoChannelManager*> pdo_channel_config;
} pdo_mapping_t;*/

typedef struct 
{
    uint16_t index; /**< PDO index. */
    unsigned int n_entries; /**< Number of PDO entries in \a entries to map. */
    PdoType pdo_type;

} pdo_info_t;




//typedef std::vector<pdo_mapping_t> pdo_config_t;

class EcSlaveBase
{
public:
  EcSlaveBase() {}
  ~EcSlaveBase() {}

  /** read or write data to the domain */
  virtual void process_data(int /* index */, uint8_t * /*domain_address*/) {}
  /*virtual int process_data(
    size_t pdo_mapping_index, size_t pdo_channel_index, uint8_t * domain_address)
  {
    pdo_config_[pdo_mapping_index].pdo_channel_config[pdo_channel_index].ec_update(domain_address);
    return 0;
  }*/
/** Assign activate DC synchronization. return activate word*/
  virtual int assign_activate_dc_sync() {return 0x00;}
  virtual bool initialized() {return true;}
  virtual void set_state_is_operational(bool value) {is_operational_ = value;}

  virtual void updateState() {}

  inline
  void setAliasAndPosition(uint16_t alias, uint16_t position)
  {
    alias_ = alias;
    position_ = position;
    is_alias_and_position_set_ = true;
  }

  inline
  bool isAliasAndPositionSet()
  {
    return is_alias_and_position_set_;
  }



  virtual bool setup_slave(
    std::unordered_map<std::string, std::string> slave_paramters,
    std::vector<double> * state_interface,
    std::vector<double> * command_interface)
  {
    state_interface_ptr_ = state_interface;
    command_interface_ptr_ = command_interface;
    parameters_ = slave_paramters;
    is_initialized_ = true;
    //setup_interface_mapping();
    return true;
  }

  uint32_t get_vendor_id() {return vendor_id_;}
  uint32_t get_product_id() {return product_id_;}
  uint16_t get_alias() {return alias_;}
  uint16_t get_position() {return position_;}

  //pdo_config_t get_pdo_config() {return pdo_config_;}

  std::vector<SMConfig> get_sm_config()
  {
    return sm_config_;
  }

  std::vector<SdoConfigEntry> get_sdo_config()
  {
    return sdo_config_;
  }

  std::vector<ethercat_interface::EcPdoChannelManager *> get_pdo_channels_info() 
  {
    return pdo_channels_info_;  
  }

  std::vector<pdo_info_t> get_pdo_info()
  {
    return pdo_info_;
  }

  /*std::unordered_map<std::string, std::string> get_slave_parameters()
  {
    return paramters_;
  }*/

  //std::vector<double> * get_state_interface_ptr()
  //{
  //  return state_interface_ptr_;
  //}

  //std::vector<double> * get_command_interface_ptr()
  //{
   // return command_interface_ptr_;
  //}

protected:
  std::vector<double> * state_interface_ptr_;
  std::vector<double> * command_interface_ptr_;
  std::unordered_map<std::string, std::string> parameters_;
  bool is_initialized_ = true;
  //bool is_operational_ = false;
  uint16_t alias_;        //< Slave alias.
  uint16_t position_;     //< Index after alias. If alias is zero, stores the ring position.
  uint32_t vendor_id_;   //< Slave vendor ID.
  uint32_t product_id_;  //< Slave product code.

  bool is_operational_ = false;
  bool is_alias_and_position_set_ = false;


  std::vector<SdoConfigEntry> sdo_config_;
  std::vector<SMConfig> sm_config_;
  std::vector<ethercat_interface::EcPdoChannelManager *> pdo_channels_info_;
  std::vector<pdo_info_t> pdo_info_;
  //pdo_config_t pdo_config_;

  //virtual void setup_interface_mapping() = 0;

  /*void setup_interface_mapping()
  {
    for (auto & mapping : pdo_config_) {
      for (auto & channel : mapping.pdo_channel_config) {
        if (channel.pdo_type == ethercat_interface::TPDO) {
          if (paramters_.find("state_interface/" + channel.interface_name) != paramters_.end()) {
            channel.interface_index =
              std::stoi(paramters_["state_interface/" + channel.interface_name]);
          }
        }
        if (channel.pdo_type == ethercat_interface::RPDO) {
          if (paramters_.find("command_interface/" + channel.interface_name) != paramters_.end()) {
            channel.interface_index = std::stoi(
              paramters_["command_interface/" + channel.interface_name]);
          }
        }

        channel.setup_interface_ptrs(state_interface_ptr_, command_interface_ptr_);
      }
    }
  }*/
};

}  // namespace ethercat_interface
#endif  // ETHERCAT_INTERFACE__EC_SLAVE_BASE_HPP_
