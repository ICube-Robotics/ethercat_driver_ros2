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
// Author: Maciej Bednarczyk (macbednarczyk@gmail.com)

#ifndef ETHERCAT_PLUGINS__GENERIC_PLUGINS__CIA402_DRIVE_HPP_
#define ETHERCAT_PLUGINS__GENERIC_PLUGINS__CIA402_DRIVE_HPP_

#include <vector>
#include <string>
#include <unordered_map>

#include "yaml-cpp/yaml.h"
#include "ethercat_interface/ec_slave.hpp"
#include "ethercat_plugins/commondefs.hpp"
#include "ethercat_plugins/ec_pdo_channel_manager.hpp"

namespace ethercat_plugins
{

class CiA402Drive : public ethercat_interface::EcSlave
{
public:
  CiA402Drive();
  virtual ~CiA402Drive();
  /** Returns true if drive has reached "operation enabled" state.
   *  The transition through the state machine is handled automatically. */
  bool initialized() const;
  virtual int assign_activate_dc_sync();

  virtual void processData(size_t index, uint8_t * domain_address);

  virtual const ec_sync_info_t * syncs();
  virtual size_t syncSize();
  virtual const ec_pdo_entry_info_t * channels();
  virtual void domains(DomainMap & domains) const;

  virtual bool setupSlave(
    std::unordered_map<std::string, std::string> slave_paramters,
    std::vector<double> * state_interface,
    std::vector<double> * command_interface);

  int8_t mode_of_operation_display_ = 0;
  int8_t mode_of_operation_ = 0;

protected:
  uint32_t counter_ = 0;
  std::vector<ec_pdo_info_t> rpdos_;
  std::vector<ec_pdo_info_t> tpdos_;
  std::vector<ec_pdo_entry_info_t> all_channels_;
  std::vector<EcPdoChannelManager> pdo_channels_info_;
  std::vector<ec_sync_info_t> syncs_;
  uint16_t last_status_word_ = -1;
  uint16_t status_word_ = 0;
  uint16_t control_word_ = 0;
  DeviceState last_state_ = STATE_START;
  DeviceState state_ = STATE_START;
  bool initialized_ = false;
  bool auto_fault_reset_ = false;
  bool fault_reset_ = false;
  int fault_reset_command_interface_index_ = -1;
  bool last_fault_reset_command_ = false;
  YAML::Node drive_config_;
  uint32_t assign_activate_ = 0;
  double last_position_ = 0;

  /** returns device state based upon the status_word */
  DeviceState deviceState(uint16_t status_word);

  /** returns the control word that will take device from state to next desired state */
  uint16_t transition(DeviceState state, uint16_t control_word);

  /** set up of the drive configuration from yaml node*/
  bool setup_from_config(YAML::Node drive_config);
  /** set up of the drive configuration from yaml file*/
  bool setup_from_config_file(std::string config_file);

  void setup_syncs();

  void setup_interface_mapping();
};
}  // namespace ethercat_plugins

#endif  // ETHERCAT_PLUGINS__GENERIC_PLUGINS__CIA402_DRIVE_HPP_
