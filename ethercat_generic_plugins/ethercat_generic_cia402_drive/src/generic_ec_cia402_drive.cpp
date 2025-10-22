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

#include <numeric>

#include "ethercat_generic_plugins/generic_ec_cia402_drive.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ethercat_generic_plugins
{

EcCiA402Drive::EcCiA402Drive()
: GenericEcSlave() {}
EcCiA402Drive::~EcCiA402Drive() {}

bool EcCiA402Drive::initialized() {return initialized_;}

void EcCiA402Drive::updateState()
{
  if (status_word_ != last_status_word_) {
    state_ = deviceState(status_word_);
    if (state_ != last_state_) {
      RCLCPP_INFO(
        rclcpp::get_logger("EthercatDriver"),
        "STATE: %s with status word :%d",
        DEVICE_STATE_STR.at(state_).c_str(),
        status_word_
      );
    }
  }
  last_status_word_ = status_word_;
  last_state_ = state_;
  counter_++;
  initialized_ = is_operational_;
}

void EcCiA402Drive::processData(size_t entry_idx, uint8_t * domain_address)
{
  auto index = domain_map_[entry_idx];
  ethercat_interface::EcPdoSingleInterfaceChannelManager * channel_ptr =
    static_cast<
    ethercat_interface::EcPdoSingleInterfaceChannelManager *>(
    pdo_channels_info_[index]);
  ethercat_interface::EcPdoSingleInterfaceChannelManager & channel(*channel_ptr);
  // Special case: ControlWord
  if (channel.index == CiA402D_RPDO_CONTROLWORD) {
    if (is_operational_) {
      if (fault_reset_command_interface_index_ >= 0) {
        if (command_interface_ptr_->at(fault_reset_command_interface_index_) == 0) {
          last_fault_reset_command_ = false;
        }
        if (last_fault_reset_command_ == false &&
          command_interface_ptr_->at(fault_reset_command_interface_index_) != 0 &&
          !std::isnan(command_interface_ptr_->at(fault_reset_command_interface_index_)))
        {
          last_fault_reset_command_ = true;
          fault_reset_ = true;
        }
      }

      if (auto_state_transitions_) {
        channel.default_value = transition(
          state_,
          channel.ec_read(domain_address));
      }
    }
  }

  // setup current position as default position
  if (channel.index == CiA402D_RPDO_POSITION) {
    if (mode_of_operation_display_ != ModeOfOperation::MODE_NO_MODE) {
      channel.default_value =
        channel.factor * last_position_ + channel.offset;
    }
    channel.override_command =
      (mode_of_operation_display_ != ModeOfOperation::MODE_CYCLIC_SYNC_POSITION) ? true : false;
  }

  // setup mode of operation
  if (channel.index == CiA402D_RPDO_MODE_OF_OPERATION) {
    if (mode_of_operation_ >= 0 && mode_of_operation_ <= 10) {
      channel.default_value = mode_of_operation_;
    }
  }

  channel.ec_update(domain_address);

  // get mode_of_operation_display_
  if (channel.index == CiA402D_TPDO_MODE_OF_OPERATION_DISPLAY) {
    mode_of_operation_display_ = channel.last_value;
  }

  if (channel.index == CiA402D_TPDO_POSITION) {
    last_position_ = channel.last_value;
  }

  // Special case: StatusWord
  if (channel.index == CiA402D_TPDO_STATUSWORD) {
    status_word_ = channel.last_value;
  }


  // CHECK FOR STATE CHANGE
  if (entry_idx == domain_map_.size() - 1) {  // if last entry in domain
    updateState();
  }
}

bool EcCiA402Drive::setupSlave(
  std::unordered_map<std::string, std::string> slave_parameters,
  std::vector<double> * state_interface,
  std::vector<double> * command_interface)
{
  state_interface_ptr_ = state_interface;
  command_interface_ptr_ = command_interface;
  parameters_ = slave_parameters;

  if (parameters_.find("slave_config") != parameters_.end()) {
    if (!setup_from_config_file(parameters_["slave_config"])) {
      return false;
    }
  } else {
    std::cerr << "EcCiA402Drive: failed to find 'slave_config' tag in URDF." << std::endl;
    return false;
  }

  setup_interface_mapping();
  setup_syncs();

  if (parameters_.find("mode_of_operation") != parameters_.end()) {
    mode_of_operation_ = std::stod(parameters_["mode_of_operation"]);
  }

  if (parameters_.find("command_interface/reset_fault") != parameters_.end()) {
    fault_reset_command_interface_index_ = std::stoi(parameters_["command_interface/reset_fault"]);
  }

  return true;
}

bool EcCiA402Drive::setup_from_config(YAML::Node drive_config)
{
  if (!GenericEcSlave::setup_from_config(drive_config)) {return false;}
  // additional configuration parameters for CiA402 Drives
  if (drive_config["auto_fault_reset"]) {
    auto_fault_reset_ = drive_config["auto_fault_reset"].as<bool>();
  }
  if (drive_config["auto_state_transitions"]) {
    auto_state_transitions_ = drive_config["auto_state_transitions"].as<bool>();
  }
  return true;
}

bool EcCiA402Drive::setup_from_config_file(std::string config_file)
{
  // Read drive configuration from YAML file
  try {
    slave_config_ = YAML::LoadFile(config_file);
  } catch (const YAML::ParserException & ex) {
    RCLCPP_ERROR(
      rclcpp::get_logger("EthercatDriver"),
      "EcCiA402Drive: failed to load drive configuration: %s",
      ex.what());
    return false;
  } catch (const YAML::BadFile & ex) {
    RCLCPP_ERROR(
      rclcpp::get_logger("EthercatDriver"),
      "EcCiA402Drive: failed to load drive configuration: %s",
      ex.what());
    return false;
  }
  if (!setup_from_config(slave_config_)) {
    return false;
  }
  return true;
}

/** returns device state based upon the status_word */
DeviceState EcCiA402Drive::deviceState(uint16_t status_word)
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
uint16_t EcCiA402Drive::transition(DeviceState state, uint16_t control_word)
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
      if (auto_fault_reset_ || fault_reset_) {
        fault_reset_ = false;
        return (control_word & 0b11111111) | 0b10000000;     // automatic reset
      } else {
        return control_word;
      }
    default:
      break;
  }
  return control_word;
}

}  // namespace ethercat_generic_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ethercat_generic_plugins::EcCiA402Drive, ethercat_interface::EcSlave)
