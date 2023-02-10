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

#include "ethercat_interface/ec_slave.hpp"
#include "ethercat_plugins/commondefs.hpp"

namespace ethercat_plugins
{

class Technosoft_IPOS : public ethercat_interface::EcSlave
{
public:
  Technosoft_IPOS()
  : EcSlave(0x000001a3, 0x01ab46e5) {}
  virtual ~Technosoft_IPOS() {}
  /** Returns true if drive has reached "operation enabled" state.
   *  The transition through the state machine is handled automatically. */
  bool initialized() const {return initialized_;}
  virtual int assign_activate_dc_sync() {return 0x0330;}

  virtual void processData(size_t index, uint8_t * domain_address)
  {
    // DATA READ WRITE
    switch (index) {
      case 0:
        control_word_ = EC_READ_U16(domain_address);
        if (is_operational_) {
          // initialization sequence
          control_word_ = transition(state_, control_word_);
        }
        EC_WRITE_U16(domain_address, control_word_);
        break;
      case 1:
        if (isTargetPositionRequired && (
            mode_of_operation_display_ == MODE_CYCLIC_SYNC_POSITION ||
            mode_of_operation_display_ == MODE_PROFILED_POSITION ||
            mode_of_operation_display_ == MODE_INTERPOLATED_POSITION ))
        {
          if (!std::isnan(command_interface_ptr_->at(cii_target_position))) {
            target_position_ = command_interface_ptr_->at(cii_target_position);
          } else {
            target_position_ = position_;
          }
        }
        EC_WRITE_S32(domain_address, target_position_);
        break;
      case 2:
        EC_WRITE_S8(domain_address, mode_of_operation_);
        break;
      case 3:
        if (isTargetVelocityRequired && (
            mode_of_operation_display_ == MODE_CYCLIC_SYNC_VELOCITY ||
            mode_of_operation_display_ == MODE_PROFILED_VELOCITY ))
        {
          if (!std::isnan(command_interface_ptr_->at(cii_target_velocity))) {
            target_velocity_ = command_interface_ptr_->at(cii_target_velocity);
          }
        }
        EC_WRITE_S32(domain_address, target_velocity_);
        break;
      case 4:
        digital_output_mask_ = 0;
        for (auto i = 0ul; i < 8; i++) {
          if (cii_do_[i] >= 0) {
            digital_output_mask_ += ( 1 << i );
          }
        }
        EC_WRITE_U8(domain_address, digital_output_mask_);
        break;
      case 5:
        digital_output_data_ = 0;
        for (auto i = 0ul; i < 8; i++) {
          if (cii_do_[i] >= 0) {
            if (!std::isnan(command_interface_ptr_->at(cii_do_[i]))) {
              write_data_[i] = (command_interface_ptr_->at(cii_do_[i])) ? true : false;
              if (sii_do_[i] >= 0) {
                state_interface_ptr_->at(sii_do_[i]) = command_interface_ptr_->at(cii_do_[i]);
              }
            }
          }
        }
        // bit masking to get individual input values
        digital_output_data_ += ( write_data_[0] << 0 );  // bit 0
        digital_output_data_ += ( write_data_[1] << 1 );  // bit 1
        digital_output_data_ += ( write_data_[2] << 2 );  // bit 2
        digital_output_data_ += ( write_data_[3] << 3 );  // bit 3
        digital_output_data_ += ( write_data_[4] << 4 );  // bit 4
        digital_output_data_ += ( write_data_[5] << 5 );  // bit 5
        digital_output_data_ += ( write_data_[6] << 6 );  // bit 6
        digital_output_data_ += ( write_data_[7] << 7 );  // bit 7
        EC_WRITE_U8(domain_address, digital_output_data_);
        break;
      case 6:
        status_word_ = EC_READ_U16(domain_address);
        state_ = deviceState(status_word_);
        break;
      case 7:
        position_ = EC_READ_S32(domain_address);
        if (isPositionRequired) {
          state_interface_ptr_->at(sii_position) = position_;
        }
        break;

      case 8:
        mode_of_operation_display_ = EC_READ_S8(domain_address);
        break;
      case 9:
        velocity_ = EC_READ_S32(domain_address);
        if (isVelocityRequired) {
          state_interface_ptr_->at(sii_velocity) = velocity_;
        }
        break;
      case 10:
        torque_ = EC_READ_S16(domain_address);
        if (isTorqueRequired) {
          state_interface_ptr_->at(sii_torque) = torque_;
        }
        break;
      case 11:
        digital_input_data_ = EC_READ_U8(domain_address);
        digital_inputs_[0] = ((digital_input_data_ & 0b00000001) != 0);    // bit 0
        digital_inputs_[1] = ((digital_input_data_ & 0b00000010) != 0);    // bit 1
        digital_inputs_[2] = ((digital_input_data_ & 0b00000100) != 0);    // bit 2
        digital_inputs_[3] = ((digital_input_data_ & 0b00001000) != 0);    // bit 3
        digital_inputs_[4] = ((digital_input_data_ & 0b00010000) != 0);    // bit 4
        digital_inputs_[5] = ((digital_input_data_ & 0b00100000) != 0);    // bit 5
        digital_inputs_[6] = ((digital_input_data_ & 0b01000000) != 0);    // bit 6
        digital_inputs_[7] = ((digital_input_data_ & 0b10000000) != 0);    // bit 7
        for (auto i = 0ul; i < 8; i++) {
          if (sii_di_[i] >= 0) {
            state_interface_ptr_->at(sii_di_[i]) = digital_inputs_[i];
          }
        }
        break;
      default:
        std::cout << "WARNING. IPOS pdo index = " << index << " out of range." << std::endl;
    }
    // CHECK FOR STATE CHANGE
    if (index == 11) {  // if last entry  in domain
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
      counter_++;
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

    if (isPositionRequired) {
      sii_position = std::stoi(
        paramters_["state_interface/position"]);
    }
    if (isVelocityRequired) {
      sii_velocity = std::stoi(
        paramters_["state_interface/velocity"]);
    }
    if (isTorqueRequired) {
      sii_torque = std::stoi(
        paramters_["state_interface/effort"]);
    }

    if (isTargetPositionRequired) {
      cii_target_position = std::stoi(
        paramters_["command_interface/position"]);
    }
    if (isTargetVelocityRequired) {
      cii_target_velocity = std::stoi(
        paramters_["command_interface/velocity"]);
    }

    for (auto index = 0ul; index < 8; index++) {
      if (paramters_.find("di." + std::to_string(index + 1)) != paramters_.end()) {
        if (paramters_.find(
            "state_interface/" + paramters_["di." + std::to_string(index + 1)]) != paramters_.end())
        {
          sii_di_[index] = std::stoi(
            paramters_["state_interface/" + paramters_["di." + std::to_string(index + 1)]]);
        }
      }
    }

    for (auto index = 0ul; index < 8; index++) {
      if (paramters_.find("do." + std::to_string(index + 1)) != paramters_.end()) {
        if (paramters_.find(
            "command_interface/" + paramters_["do." + std::to_string(index + 1)]) !=
          paramters_.end())
        {
          cii_do_[index] = std::stoi(
            paramters_["command_interface/" + paramters_["do." + std::to_string(index + 1)]]);
        }
        if (paramters_.find(
            "state_interface/" + paramters_["do." + std::to_string(index + 1)]) != paramters_.end())
        {
          sii_do_[index] = std::stoi(
            paramters_["state_interface/" + paramters_["do." + std::to_string(index + 1)]]);
        }
      }
    }

    return true;
  }

  int32_t target_position_ = 0;              // write
  int32_t target_velocity_ = 0;              // write
  int16_t target_torque_ = 0;                // write (max torque (max current) = 1000)
  uint16_t control_word_ = 0;                // write
  int8_t mode_of_operation_ = 0;             // write (use enum ModeOfOperation for convenience)
  int16_t torque_offset_ = 0;                // write
  int32_t position_offset_ = 0;              // write
  int32_t velocity_offset_ = 0;              // write
  uint16_t touch_probe_function_ = 0;        // write
  uint16_t dig_input_function_ = 0;          // write

  int32_t velocity_ = 0;                     // read
  int32_t position_ = 0;                     // read
  int16_t torque_ = 0;                       // read
  uint16_t status_word_ = 0;                 // read
  int8_t mode_of_operation_display_ = 0;     // read

  bool isPositionRequired = false;
  bool isVelocityRequired = false;
  bool isTorqueRequired = false;

  bool isTargetPositionRequired = false;
  bool isTargetVelocityRequired = false;

  int sii_position;
  int sii_velocity;
  int sii_torque;

  int cii_target_position;
  int cii_target_velocity;
  int cii_target_torque;

private:
  uint32_t counter_ = 0;
  uint8_t digital_output_ = 0;  // write
  uint8_t digital_output_mask_ = 0;  // 0b00000011 (use the 2 first general purpose digital out)
  uint8_t digital_input_data_ = 0;  // read digital input
  uint8_t digital_output_data_ = 0; // write digital output
  bool digital_inputs_[8];
  int sii_di_[8] = {-1, -1, -1, -1, -1, -1, -1, -1};
  int cii_do_[8] = {-1, -1, -1, -1, -1, -1, -1, -1};
  int sii_do_[8] = {-1, -1, -1, -1, -1, -1, -1, -1};
  bool write_data_[8] = {false, false, false, false, false, false, false, false};

  ec_pdo_entry_info_t channels_[12] = {
    {0x6040, 0x00, 16},  /* 0. Control word */
    {0x607a, 0x00, 32},  /* 1. Target position */

    {0x6060, 0x00, 8},  /* 2. Mode of operation */
    {0x60ff, 0x00, 32},  /* 3. Target velocity */

    {0x2090, 0x02, 8},  /* 4. Digital outputs mask */
    {0x2090, 0x01, 8},  /* 5. Digital outputs value */

    {0x6041, 0x00, 16},  /* 6. Statusword */
    {0x6064, 0x00, 32},  /* 7. Position actual value */

    {0x6061, 0x00, 8},  /* 8. Mode of operation display */
    {0x606c, 0x00, 32},  // 9. Velocity actual value  65536 = 1 encoder
                         // increment/sample (internal driver slow loop : 1khz )

    {0x6077, 0x00, 16},  /* 10. Torque actual value */
    {0x208F, 0x02, 8}  /* 11. Digital inputs value */
  };
  ec_pdo_info_t pdos_[6] = {
    {0x1600, 2, channels_ + 0},  /* RPDO0 Mapping */
    {0x1601, 2, channels_ + 2},  /* RPDO1 Mapping */
    {0x1602, 2, channels_ + 4},  /* RPDO1 Mapping */
    {0x1a00, 2, channels_ + 6},  /* TPDO0 Mapping */
    {0x1a01, 2, channels_ + 8},  /* TPDO1 Mapping */
    {0x1a02, 2, channels_ + 10},  /* TPDO2 Mapping */
  };
  ec_sync_info_t syncs_[5] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 3, pdos_, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 3, pdos_ + 3, EC_WD_DISABLE},
    {0xff}
  };
  DomainMap domains_ = {
    {0, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}}
  };
//========================================================
// Technosoft SPECIFIC
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
      case STATE_START:                     // -> STATE_NOT_READY_TO_SWITCH_ON (automatic)
        return control_word;
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
        // return control_word;
        return (control_word & 0b11111111) | 0b10000000;     // automatic reset
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

PLUGINLIB_EXPORT_CLASS(ethercat_plugins::Technosoft_IPOS, ethercat_interface::EcSlave)
