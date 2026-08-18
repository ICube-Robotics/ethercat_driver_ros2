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

#ifndef ETHERCAT_GENERIC_PLUGINS__GENERIC_EC_CIA402_DRIVE_HPP_
#define ETHERCAT_GENERIC_PLUGINS__GENERIC_EC_CIA402_DRIVE_HPP_

#include <vector>
#include <string>
#include <unordered_map>
#include <limits>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "yaml-cpp/yaml.h"
#include "ethercat_interface/ec_slave_base.hpp"
#include "ethercat_interface/ec_pdo_single_interface_channel_manager.hpp"
#include "ethercat_generic_plugins/generic_ec_slave.hpp"
#include "ethercat_generic_plugins/cia402_common_defs.hpp"

namespace ethercat_generic_plugins
{

class EcCiA402Drive : public GenericEcSlave
{
public:
  EcCiA402Drive();
  virtual ~EcCiA402Drive();
  /** Returns true if drive has reached "operation enabled" state.
   *  The transition through the state machine is handled automatically. */
  bool initialized();

  virtual void process_data(int index, uint8_t * domain_address);

  virtual bool setup_slave(
    std::unordered_map<std::string, std::string> slave_parameters,
    std::vector<double> * state_interface,
    std::vector<double> * command_interface);

  int8_t mode_of_operation_display_ = 0;
  int8_t mode_of_operation_ = -1;

  void updateState();

  /** Appends CiA402 device state / mode of operation / (while faulted) error code to
   *  GenericEcSlave's generic online/al_state diagnostics. See EcSlaveBase::collectDiagnostics()
   *  — call the base first, this only adds to it. */
  virtual void collectDiagnostics(diagnostic_msgs::msg::DiagnosticStatus & status) const;

protected:
  /** \brief Whether this cycle's commanded target position (0x607a) passes through
    * to the drive (true) or is overridden with the actual position (false). The
    * stock plugin passes it through only in Cyclic Synchronous Position mode;
    * vendor subclasses may extend this to other modes that use 0x607a as a
    * position/equilibrium setpoint (e.g. an impedance-style vendor mode, 0x6060 = -6). */
  virtual bool targetPositionPassthrough() const;

  uint32_t counter_ = 0;
  uint16_t last_status_word_ = -1;
  uint16_t status_word_ = 0;
  uint16_t control_word_ = 0;
  // CoE 0x603F, only meaningful once faulted. Stays 0 if the slave_config doesn't map it.
  uint16_t error_code_ = 0;
  DeviceState last_state_ = STATE_START;
  DeviceState state_ = STATE_START;
  bool initialized_ = false;
  bool auto_fault_reset_ = false;
  bool auto_state_transitions_ = true;
  bool fault_reset_ = false;
  int fault_reset_command_interface_index_ = -1;
  bool last_fault_reset_command_ = false;
  double last_position_ = std::numeric_limits<double>::quiet_NaN();

  // enable_drive/disable_drive: edge-triggered command interfaces (no PDO object of their
  // own) that walk the drive toward Operation Enabled / Switch-on-Disabled over as many
  // cycles as it takes, independently of auto_state_transitions_. Meant for a caller that
  // wants explicit control without hand-rolling the CiA402 walk itself.
  int enable_drive_command_interface_index_ = -1;
  int disable_drive_command_interface_index_ = -1;
  bool last_enable_drive_command_ = false;
  bool last_disable_drive_command_ = false;
  bool walking_to_enabled_ = false;
  bool walking_to_disabled_ = false;

  // True for one cycle after a fault_reset_ pulse is sent while nothing else
  // (auto_state_transitions_) keeps calling transition() on later cycles — that path has no
  // other opportunity to clear the pulse back down, so process_data() clears it directly. See
  // its ControlWord handling.
  bool fault_reset_pulse_active_ = false;

  /** returns device state based upon the status_word */
  DeviceState deviceState(uint16_t status_word);
  /** returns the control word that will take device from state to next desired state */
  uint16_t transition(DeviceState state, uint16_t control_word);
  /** set up of the drive configuration from yaml node*/
  bool setup_from_config(YAML::Node drive_config);
  /** set up of the drive configuration from yaml file*/
  bool setup_from_config_file(std::string config_file);
};
}  // namespace ethercat_generic_plugins

#endif  // ETHERCAT_GENERIC_PLUGINS__GENERIC_EC_CIA402_DRIVE_HPP_
