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

#include "ethercat_interface/ec_slave.hpp"
#include "ethercat_maxon_drives/maxon_defs.hpp"

namespace ethercat_plugins
{

class Maxon_EPOS3 : public ethercat_interface::EcSlave
{
public:
  Maxon_EPOS3()
  : EcSlave(0x000000fb, 0x64400000)
  {
    std::cerr << "The Maxon_EPOS3 plugin is depreciated and will be removed in the future."
              << "Use the EcCiA402Drive plugin instead." << std::endl;
  }
  virtual ~Maxon_EPOS3() {}
  /** Returns true if Epos3 has reached "operation enabled" state.
    *  The transition through the state machine is handled automatically. */
  bool initialized() const {return initialized_;}
  virtual int assign_activate_dc_sync() {return 0x300;}

  virtual void processData(size_t index, uint8_t * domain_address)
  {
    // DATA READ WRITE
    switch (index) {
      case 0:
        control_word_ = EC_READ_U16(domain_address);
        // initialization sequence
        control_word_ = transition(state_, control_word_);
        EC_WRITE_U16(domain_address, control_word_);
        break;
      case 1:
        if (isTargetPositionRequired && (
            mode_of_operation_display_ == MODE_CYCLIC_SYNC_POSITION ||
            mode_of_operation_display_ == MODE_PROFILED_POSITION ||
            mode_of_operation_display_ == MODE_INTERPOLATED_POSITION ))
        {
          target_position_ = command_interface_ptr_->at(cii_target_position);
        }
        EC_WRITE_S32(domain_address, target_position_);
        break;
      case 2:
        if (isTargetVelocityRequired && (
            mode_of_operation_display_ == MODE_CYCLIC_SYNC_VELOCITY ||
            mode_of_operation_display_ == MODE_PROFILED_VELOCITY ))
        {
          target_velocity_ = command_interface_ptr_->at(cii_target_velocity);
        }
        EC_WRITE_S32(domain_address, target_velocity_);
        break;
      case 3:
        if (isTargetTorqueRequired && (
            mode_of_operation_display_ == MODE_CYCLIC_SYNC_TORQUE ||
            mode_of_operation_display_ == MODE_PROFILED_TORQUE ))
        {
          target_torque_ = command_interface_ptr_->at(cii_target_torque);
        }
        EC_WRITE_S16(domain_address, target_torque_);
        break;
      case 4:
        EC_WRITE_S32(domain_address, position_offset_);
        break;
      case 5:
        EC_WRITE_S32(domain_address, velocity_offset_);
        break;
      case 6:
        EC_WRITE_S16(domain_address, torque_offset_);
        break;
      case 7:
        EC_WRITE_S8(domain_address, mode_of_operation_);
        break;
      case 8:
        EC_WRITE_U16(domain_address, dig_input_function_);
        break;
      case 9:
        EC_WRITE_U16(domain_address, touch_probe_function_);
        break;
      case 10:
        status_word_ = EC_READ_U16(domain_address);
        state_ = deviceState(status_word_);
        break;
      case 11:
        position_ = EC_READ_S32(domain_address);
        if (isPositionRequired) {
          state_interface_ptr_->at(sii_position) = position_;
        }
        break;
      case 12:
        velocity_ = EC_READ_S32(domain_address);
        if (isVelocityRequired) {
          state_interface_ptr_->at(sii_velocity) = velocity_;
        }
        break;
      case 13:
        torque_ = EC_READ_S16(domain_address);
        if (isTorqueRequired) {
          state_interface_ptr_->at(sii_torque) = torque_;
        }
        break;
      case 14:
        mode_of_operation_display_ = EC_READ_S8(domain_address);
        break;
      case 15:
        break;
      case 16:
        break;
      case 17:
        break;
      case 18:
        break;
      default:
        std::cout << "WARNING. Epos3 pdo index = " << index << " out of range." << std::endl;
    }
    // CHECK FOR STATE CHANGE
    if (index == 18) {  // if last entry  in domain
      if (status_word_ != last_status_word_) {
        state_ = deviceState(status_word_);
        // status word change does not necessarily mean state change
        // http://ftp.beckhoff.com/download/document/motion/ax2x00_can_manual_en.pdf
        // std::bitset<16> temp(status_word_);
        // std::cout << "STATUS WORD: " << temp << std::endl;
        if (state_ != last_state_) {
          std::cout << "STATE: " << DEVICE_STATE_STR.at(state_)
                    << " with status word :" << status_word_ << std::endl;
        }
      }
      if ((state_ == STATE_OPERATION_ENABLED) &&
        (last_state_ == STATE_OPERATION_ENABLED))
      {
        initialized_ = true;
      } else {
        initialized_ = false;
      }
      last_status_word_ = status_word_;
      last_state_ = state_;
    }
  }

  virtual const ec_sync_info_t * syncs() {return &syncs_[0];}
  virtual size_t syncSize()
  {
    return sizeof(syncs_) / sizeof(ec_sync_info_t);
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
    if (paramters_.find("mode_of_operation") != paramters_.end()) {
      mode_of_operation_ = std::stod(paramters_["mode_of_operation"]);
    }

    isPositionRequired = paramters_.find("state_interface/position") != paramters_.end();
    isVelocityRequired = paramters_.find("state_interface/velocity") != paramters_.end();
    isTorqueRequired = paramters_.find("state_interface/effort") != paramters_.end();

    isTargetPositionRequired = paramters_.find("command_interface/position") != paramters_.end();
    isTargetVelocityRequired = paramters_.find("command_interface/velocity") != paramters_.end();
    isTargetTorqueRequired = paramters_.find("command_interface/effort") != paramters_.end();

    if (isPositionRequired) {
      sii_position = std::stoi(paramters_["state_interface/position"]);
    }
    if (isVelocityRequired) {
      sii_velocity = std::stoi(paramters_["state_interface/velocity"]);
    }
    if (isTorqueRequired) {
      sii_torque = std::stoi(paramters_["state_interface/effort"]);
    }
    if (isTargetPositionRequired) {
      cii_target_position = std::stoi(paramters_["command_interface/position"]);
    }
    if (isTargetVelocityRequired) {
      cii_target_velocity = std::stoi(paramters_["command_interface/velocity"]);
    }
    if (isTargetTorqueRequired) {
      cii_target_torque = std::stoi(paramters_["command_interface/effort"]);
    }

    return true;
  }

  int32_t target_position_ = 0;   // write
  int32_t target_velocity_ = 0;   // write
  int16_t target_torque_ = 0;   // write (max torque (max current) = 1000)
  uint16_t control_word_ = 0;  // write
  int8_t mode_of_operation_ = 0;    // write (use enum ModeOfOperation for convenience)
  int16_t torque_offset_ = 0;   // write
  int32_t position_offset_ = 0;   // write
  int32_t velocity_offset_ = 0;   // write
  uint16_t touch_probe_function_ = 0;  // write
  uint16_t dig_input_function_ = 0;  // write

  int32_t velocity_ = 0;   // read
  int32_t position_ = 0;   // read
  int16_t torque_ = 0;   // read
  uint16_t status_word_ = 0;  // read
  int8_t mode_of_operation_display_ = 0;    // read

  bool isPositionRequired = false;
  bool isVelocityRequired = false;
  bool isTorqueRequired = false;

  bool isTargetPositionRequired = false;
  bool isTargetVelocityRequired = false;
  bool isTargetTorqueRequired = false;

  int sii_position;
  int sii_velocity;
  int sii_torque;

  int cii_target_position;
  int cii_target_velocity;
  int cii_target_torque;

private:
  uint32_t digital_output_ = 0;  // write
  uint32_t digital_input_ = 0;  // read (must be enabled in Epos3 Motion Studio)
  ec_pdo_entry_info_t channels_[19] = {
    {0x6040, 0x00, 16},  /* 0 Control word */
    {0x607a, 0x00, 32},  /* 1 Target Position */
    {0x60ff, 0x00, 32},  /* 2 Target Velocity */
    {0x6071, 0x00, 16},  /* 3 Target Torque */
    {0x60b0, 0x00, 32},  /* 4 Position Offset */
    {0x60b1, 0x00, 32},  /* 5 Velocity Offset */
    {0x60b2, 0x00, 16},  /* 6 Torque Offset */
    {0x6060, 0x00, 8},   /* 7 Modes of Operation */
    {0x2078, 0x01, 16},  /* 8 Digital Output Functionalities */
    {0x60b8, 0x00, 16},  /* 9 Touch Probe Function */
    {0x6041, 0x00, 16},  /* 10 Status word */
    {0x6064, 0x00, 32},  /* 11 Position Actual Value */
    {0x606c, 0x00, 32},  /* 12 Velocity Actual Value */
    {0x6077, 0x00, 16},  /* 13 Torque Actual Value */
    {0x6061, 0x00, 8},   /* 14 Mode of operation display */
    {0x2071, 0x01, 16},  /* 15 Digital Input Functionalities State */
    {0x60b9, 0x00, 16},  /* 16 Touch Probe Status */
    {0x60ba, 0x00, 32},  /* 17 Touch Probe Position 1 Positive Value */
    {0x60bb, 0x00, 32},  /* 18 Touch Probe Position 1 Negative Value */
  };
  ec_pdo_info_t pdos_[2] = {
    {0x1603, 10, channels_ + 0},  /* 4th receive PDO Mapping */
    {0x1a03, 9, channels_ + 10},  /* 4th transmit PDO Mapping */
  };
  ec_sync_info_t syncs_[5] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, pdos_, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, pdos_ + 1, EC_WD_DISABLE},
    {0xff}
  };
  DomainMap domains_ = {
    {0, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18}}
  };
//========================================================
// Epos3 SPECIFIC
//========================================================

  /** returns device state based upon the status_word */
  DeviceState deviceState(uint16_t status_word)
  {
    if ((status_word & 0b01001111) == 0b00000000) {
      return STATE_NOT_READY_TO_SWITCH_ON;
    } else if ((status_word & 0b01001111) == 0b01000000) {
      return STATE_SWITCH_ON_DISABLED;
    } else if ((status_word & 0b01101111) == 0b00100001) {
      return STATE_READY_TO_SWITCH_ON;
    } else if ((status_word & 0b01101111) == 0b00100011) {
      return STATE_SWITCH_ON;
    } else if ((status_word & 0b01101111) == 0b00100111) {
      return STATE_OPERATION_ENABLED;
    } else if ((status_word & 0b01101111) == 0b00000111) {
      return STATE_QUICK_STOP_ACTIVE;
    } else if ((status_word & 0b01001111) == 0b00001111) {
      return STATE_FAULT_REACTION_ACTIVE;
    } else if ((status_word & 0b01001111) == 0b00001000) {
      return STATE_FAULT;
    }
    return STATE_UNDEFINED;
  }
  /** returns the control word that will take device from state to next desired state */
  uint16_t transition(DeviceState state, uint16_t control_word)
  {
    switch (state) {
      case STATE_START:  // -> STATE_NOT_READY_TO_SWITCH_ON (automatic)
        return control_word;
      case STATE_NOT_READY_TO_SWITCH_ON:  // -> STATE_SWITCH_ON_DISABLED (automatic)
        return control_word;
      case STATE_SWITCH_ON_DISABLED:  // -> STATE_READY_TO_SWITCH_ON
        return (control_word & 0b01111110) | 0b00000110;
      case STATE_READY_TO_SWITCH_ON:  // -> STATE_SWITCH_ON
        return (control_word & 0b01110111) | 0b00000111;
      case STATE_SWITCH_ON:  // -> STATE_OPERATION_ENABLED
        return (control_word & 0b01111111) | 0b00001111;
      case STATE_OPERATION_ENABLED:  // -> GOOD
        return control_word;
      case STATE_QUICK_STOP_ACTIVE:  // -> STATE_OPERATION_ENABLED
        return (control_word & 0b01111111) | 0b00001111;
      case STATE_FAULT_REACTION_ACTIVE:  // -> STATE_FAULT (automatic)
        return control_word;
      case STATE_FAULT:  // -> STATE_SWITCH_ON_DISABLED
        return (control_word & 0b11111111) | 0b10000000;
      default:
        break;
    }
    return control_word;
  }
  int last_status_word_ = -1;
  DeviceState last_state_ = STATE_START;
  DeviceState state_ = STATE_START;
  bool initialized_ = false;
};
}  // namespace ethercat_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ethercat_plugins::Maxon_EPOS3, ethercat_interface::EcSlave)
