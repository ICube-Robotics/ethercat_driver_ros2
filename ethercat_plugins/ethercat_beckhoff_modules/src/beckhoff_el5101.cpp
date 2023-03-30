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

#define NULL_CMD                    ((uint16_t) 0x00)
#define WRITE_REQUEST               ((uint16_t) 0x01)

namespace ethercat_plugins
{

class Beckhoff_EL5101 : public ethercat_interface::EcSlave
{
public:
  Beckhoff_EL5101()
  : EcSlave(0x00000002, 0x13ed3052) {}
  virtual ~Beckhoff_EL5101() {}
  virtual void processData(size_t index, uint8_t * domain_address)
  {
    switch (index) {
      case 0:
        control_word_ = EC_READ_U8(domain_address);
        control_word_ = transition(state_, control_word_);
        if (cii_reset_ >= 0) {
          reset_command_ = command_interface_ptr_->at(cii_reset_);
          if (sii_reset_ >= 0) {
            state_interface_ptr_->at(sii_reset_) = reset_command_;
          }
          if (reset_command_ == 1.0 && last_reset_command_ != 1.0) {
            cmd_ = CMD_WRITE_REQUEST;
          }
          last_reset_command_ = reset_command_;
        }
        control_word_ = process_cmd(state_, cmd_, control_word_);
        last_control_word_ = control_word_;
        EC_WRITE_U8(domain_address, control_word_);
        break;
      case 1:
        if (write_t) {
          EC_WRITE_S32(domain_address, target_position_);
          write_t = false;
        }
        break;
      case 2:
        status_word_ = EC_READ_U16(domain_address);
        break;
      case 3:
        position_ = EC_READ_S32(domain_address);
        if (sii_position_ >= 0) {
          state_interface_ptr_->at(sii_position_) =
            static_cast<double>(position_) * conversion_factor_;
        }
        break;
      case 4:
        latch_ = EC_READ_S32(domain_address);
        break;
      default:
        break;
    }

    if (index == 4) {
      if (static_cast<uint8_t>(status_word_) != static_cast<uint8_t>(last_status_word_)) {
        state_ = deviceState(status_word_);
      }
      last_status_word_ = status_word_;
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
    if (paramters_.find("conversion_factor") != paramters_.end()) {
      conversion_factor_ = std::stod(paramters_["conversion_factor"]);
    }
    if (paramters_.find("encoder_reset") != paramters_.end()) {
      if (paramters_.find(
          "command_interface/" + paramters_["encoder_reset"]) != paramters_.end())
      {
        cii_reset_ = std::stoi(
          paramters_["command_interface/" + paramters_["encoder_reset"]]);
      }
      if (paramters_.find(
          "state_interface/" + paramters_["encoder_reset"]) != paramters_.end())
      {
        sii_reset_ = std::stoi(
          paramters_["state_interface/" + paramters_["encoder_reset"]]);
      }
    }
    if (paramters_.find("encoder_position") != paramters_.end()) {
      if (paramters_.find(
          "state_interface/" + paramters_["encoder_position"]) != paramters_.end())
      {
        sii_position_ = std::stoi(
          paramters_["state_interface/" + paramters_["encoder_position"]]);
      }
    }
    return true;
  }

private:
  int32_t target_position_ = 0;   // write
  uint8_t control_word_ = 0;  // write
  uint32_t latch_ = 0;  // read
  int32_t position_ = 0;   // read
  uint16_t status_word_ = 0;  // read
  uint16_t last_status_word_ = -1;
  uint8_t last_control_word_ = -1;
  double last_reset_command_ = 0.0;
  double reset_command_ = 0.0;
  bool write_ = false;  // write
  bool write_t = false;  // write
  bool error_ = false;  // read
  uint8_t cmd_ = 0;
  uint8_t error_type_ = 0;  // read
  double conversion_factor_ = 1;
  int cii_reset_ = -1;
  int sii_reset_ = -1;
  int sii_position_ = -1;


  ec_pdo_entry_info_t channels_[25] = {
    {0x7010, 0x01, 1},
    {0x7010, 0x02, 1},
    {0x7010, 0x03, 1},
    {0x7010, 0x04, 1},
    {0x0000, 0x00, 4},  // Gap
    {0x0000, 0x00, 8},  // Gap
    {0x7010, 0x11, 32},
    {0x6010, 0x01, 1},
    {0x6010, 0x02, 1},
    {0x6010, 0x03, 1},
    {0x6010, 0x04, 1},
    {0x6010, 0x05, 1},
    {0x6010, 0x06, 1},
    {0x6010, 0x07, 1},
    {0x6010, 0x08, 1},
    {0x6010, 0x09, 1},
    {0x6010, 0x0a, 1},
    {0x6010, 0x0b, 1},
    {0x6010, 0x0c, 1},
    {0x6010, 0x0d, 1},
    {0x1c32, 0x20, 1},
    {0x1804, 0x07, 1},
    {0x1804, 0x09, 1},
    {0x6010, 0x11, 32},
    {0x6010, 0x12, 32},
  };
  ec_pdo_info_t pdos_[2] = {
    {0x1603, 7, channels_ + 0},
    {0x1a04, 18, channels_ + 7},
  };
  ec_sync_info_t syncs_[5] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, pdos_ + 0, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 1, pdos_ + 1, EC_WD_DISABLE},
    {0xff}
  };

  DomainMap domains_ = {
    {0, {0, 6, 7, 23, 24}}
  };

  enum DeviceState
  {
    STATE_UNDEFINED = 0,
    STATE_READY_TO_WRITE = 1,
    STATE_NOTREADY_TO_WRITE = 2,
    STATE_WRITE = 2,
    STATE_FAULT = 3
  };

  enum CmdOfOperation
  {
    CMD_NO_CMD = NULL_CMD,
    CMD_WRITE_REQUEST = WRITE_REQUEST,
  };

  // returns the control word that will take device from state to next desired state
  uint8_t transition(DeviceState state, uint8_t control_word)
  {
    switch (state) {
      case STATE_READY_TO_WRITE:
        if (((control_word >> 2) & 1) && !write_) {
          control_word = (control_word & 0xfb);
        }
        return control_word;
      default:
        break;
    }
    return control_word;
  }

  DeviceState deviceState(uint16_t status_word)
  {
    if (((status_word ) & 0b01000000) == 0b01000000) {
      error_ = true;
      error_type_ = 3;
      return STATE_FAULT;
    }
    if ((((status_word ) & 0b00100100) == 0b00100000) ) {
      return STATE_READY_TO_WRITE;
    }
    if (((status_word ) & 0b00100100) == 0b00100100) {
      write_ = false;
      return STATE_NOTREADY_TO_WRITE;
    }
    return STATE_UNDEFINED;
  }

  uint16_t process_cmd(DeviceState state, uint8_t cmd, uint8_t control_word)
  {
    if (cmd > CMD_NO_CMD) {
      switch (state) {
        case STATE_READY_TO_WRITE:
          switch (cmd) {
            case CMD_WRITE_REQUEST:
              if ((control_word >> 2) & 1) {
                control_word = (control_word & 0xfb);
                break;
              }
              control_word = (control_word | 0x04);
              write_t = true;
              write_ = true;
              cmd_ = CMD_NO_CMD;
              break;
            default:
              cmd_ = CMD_NO_CMD;
          }
          break;
        case STATE_NOTREADY_TO_WRITE:
          switch (cmd) {
            case CMD_WRITE_REQUEST:
              if ((control_word >> 2) & 1) {
                control_word = (control_word & 0xfb);
              }
              break;
            default:
              cmd_ = CMD_NO_CMD;
          }
          break;
        default:
          cmd_ = CMD_NO_CMD;
          break;
      }
    }
    return control_word;
  }

  DeviceState state_ = STATE_NOTREADY_TO_WRITE;
};
}  // namespace ethercat_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ethercat_plugins::Beckhoff_EL5101, ethercat_interface::EcSlave)
