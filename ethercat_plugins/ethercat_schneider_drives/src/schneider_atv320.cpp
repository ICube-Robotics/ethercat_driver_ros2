// Copyright 2023 ICube-Robotics
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

#include "ethercat_interface/ec_slave.hpp"
#include "ethercat_schneider_drives/schneider_defs.hpp"

namespace ethercat_plugins
{

class Schneider_ATV320 : public ethercat_interface::EcSlave
{
public:
  Schneider_ATV320()
  : EcSlave(0x0800005a, 0x00000389) {}
  virtual ~Schneider_ATV320() {}
  /* Returns true if atv320 has reached "operation enabled" state. */
  bool initialized() const {return state_ == STATE_OPERATION_ENABLED;}
  virtual bool use_dc_sync() {return true;}

  virtual void processData(size_t index, uint8_t * domain_address)
  {
    switch (index) {
      case 0:       // Control word
        control_word_ = EC_READ_U16(domain_address);
        control_word_ = transition(state_, control_word_);
        EC_WRITE_U16(domain_address, control_word_);
        break;
      case 1:       // Target velocity
        target_velocity_ = command_interface_ptr_->at(cii_target_velocity);
        EC_WRITE_S16(domain_address, target_velocity_);
        break;
      case 2:       // Status word
        status_word_ = EC_READ_U16(domain_address);
        state_ = deviceState(status_word_);
        break;
      case 3:       // Current velocity
        // Read
        velocity_ = EC_READ_S16(domain_address);
        state_interface_ptr_->at(sii_velocity) = velocity_;
        break;
      default:
        throw std::runtime_error("ATV320 pdo out of range");
    }
  }

  virtual const ec_sync_info_t * syncs() {return &syncs_[0];}
  virtual size_t syncSize()
  {
    return 5;
  }
  virtual const ec_pdo_entry_info_t * channels()
  {
    return channels_;
  }
  virtual void domains(DomainMap & domains) const
  {
    domains = domains_;
  }

  virtual bool setupSlave(
    std::unordered_map<std::string, std::string> slave_paramters,
    std::vector<double> * state_interface,
    std::vector<double> * command_interface)
  {
    state_interface_ptr_ = state_interface;
    command_interface_ptr_ = command_interface;
    paramters_ = slave_paramters;

    sii_velocity = std::stoi(paramters_["state_interface/velocity"]);
    cii_target_velocity = std::stoi(paramters_["command_interface/velocity"]);

    return true;
  }

  // CIA - 402 Velocity mode parameters.
  uint16_t control_word_ = 0;                // write
  uint16_t status_word_ = 0;                 // read
  int sii_velocity;
  int cii_target_velocity;

  int32_t target_velocity_ = 0;              // write
  int32_t velocity_ = 0;                     // read

private:
  ec_pdo_entry_info_t channels_[4] = {
    {0x6040, 0x00, 16},     // 0 CMD - Control word
    {0x6042, 0x00, 16},     // 1 LFR - Reference speed
    {0x6041, 0x00, 16},     // 2 ETA - Status word
    {0x6044, 0x00, 16},     // 3 RFR - Current speed
  };

  ec_pdo_info_t pdos_[2] = {
    {0x1600, 2, channels_ + 0},
    {0x1a00, 2, channels_ + 2},
  };

  ec_sync_info_t syncs_[5] = {
    {0, EC_DIR_OUTPUT, 0},
    {1, EC_DIR_INPUT, 0},
    {2, EC_DIR_OUTPUT, 1, pdos_ + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, pdos_ + 1, EC_WD_DISABLE},
    {0xff}
  };

  DomainMap domains_ = {
    {0, {0, 1, 2, 3}}
  };
//========================================================
// CIA402 - Specific
//========================================================
  enum DeviceState
  {
    STATE_NOT_READY_TO_SWITCH_ON,
    STATE_SWITCH_ON_DISABLED,
    STATE_READY_TO_SWITCH_ON,
    STATE_SWITCH_ON,
    STATE_OPERATION_ENABLED,
    STATE_QUICK_STOP_ACTIVE,
    STATE_FAULT_REACTION_ACTIVE,
    STATE_FAULT
  };
  std::map<DeviceState, std::string> device_state_str_ = {
    {STATE_NOT_READY_TO_SWITCH_ON, "Not Ready to Switch On"},
    {STATE_SWITCH_ON_DISABLED, "Switch on Disabled"},
    {STATE_READY_TO_SWITCH_ON, "Ready to Switch On"},
    {STATE_SWITCH_ON, "Switch On"},
    {STATE_OPERATION_ENABLED, "Operation Enabled"},
    {STATE_QUICK_STOP_ACTIVE, "Quick Stop Active"},
    {STATE_FAULT_REACTION_ACTIVE, "Fault Reaction Active"},
    {STATE_FAULT, "Fault"}
  };

  /** returns device state based upon the status_word */
  DeviceState deviceState(uint16_t status_word)
  {
    switch (status_word & 0b01001111) {
      case 0b00000000:
        return STATE_NOT_READY_TO_SWITCH_ON;
      case 0b01000000:
        return STATE_SWITCH_ON_DISABLED;
      case 0b00001111:
        return STATE_FAULT_REACTION_ACTIVE;
      case 0b00001000:
        return STATE_FAULT;
      default:
        switch (status_word & 0b01101111) {
          case 0b00100001:
            return STATE_READY_TO_SWITCH_ON;
          case 0b00100011:
            return STATE_SWITCH_ON;
          case 0b00100111:
            return STATE_OPERATION_ENABLED;
          case 0b00000111:
            return STATE_QUICK_STOP_ACTIVE;
          default:
            throw std::runtime_error("Undefined state hit ATV320");
        }
    }
  }
  /** returns the control word that will take device from state to next desired state */
  uint16_t transition(DeviceState state, uint16_t control_word)
  {
    switch (state) {
      case STATE_NOT_READY_TO_SWITCH_ON:    // -> STATE_SWITCH_ON_DISABLED (automatic)
        return control_word;
      case STATE_SWITCH_ON_DISABLED:        // -> STATE_READY_TO_SWITCH_ON
        return (control_word & 0b01111110) | 0b00000110;
      case STATE_READY_TO_SWITCH_ON:        // -> STATE_SWITCH_ON
        return (control_word & 0b01110111) | 0b00000111;
      case STATE_SWITCH_ON:                 // -> STATE_OPERATION_ENABLED
        return (control_word & 0b01111111) | 0b00001111;
      case STATE_OPERATION_ENABLED:         // -> GOOD
        return control_word;
      case STATE_QUICK_STOP_ACTIVE:         // -> STATE_OPERATION_ENABLED
        return (control_word & 0b01111111) | 0b00001111;
      case STATE_FAULT_REACTION_ACTIVE:     // -> STATE_FAULT (automatic)
        return control_word;
      case STATE_FAULT:                     // -> STATE_SWITCH_ON_DISABLED
        return (control_word & 0b11111111) | 0b10000000;
      default:
        break;
    }
    return control_word;
  }
  DeviceState state_ = STATE_NOT_READY_TO_SWITCH_ON;
};
}  // namespace ethercat_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ethercat_plugins::Schneider_ATV320, ethercat_interface::EcSlave)
