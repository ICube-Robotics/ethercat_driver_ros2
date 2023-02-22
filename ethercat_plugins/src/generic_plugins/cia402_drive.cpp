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

#include "ethercat_plugins/generic_plugins/cia402_drive.hpp"

namespace ethercat_plugins
{

CiA402Drive::CiA402Drive()
: EcSlave(0, 0) {}
CiA402Drive::~CiA402Drive() {}
/** Returns true if drive has reached "operation enabled" state.
  *  The transition through the state machine is handled automatically. */
bool CiA402Drive::initialized() const {return initialized_;}
int CiA402Drive::assign_activate_dc_sync() {return assign_activate_;}

void CiA402Drive::processData(size_t index, uint8_t * domain_address)
{
  // Special case: ControlWord
  if (pdo_channels_info_[index].index == CiA402D_TPDO_CONTROLWORD) {
    if (is_operational_) {
      if (fault_reset_command_interface_index_ >= 0) {
        if (command_interface_ptr_->at(fault_reset_command_interface_index_) == 0) {
          last_fault_reset_command_ = false;
        }
        if (last_fault_reset_command_ == false &&
          command_interface_ptr_->at(fault_reset_command_interface_index_))
        {
          last_fault_reset_command_ = true;
          fault_reset_ = true;
        }
      }

      pdo_channels_info_[index].ec_read(domain_address);
      control_word_ = pdo_channels_info_[index].last_value;
      control_word_ = transition(state_, control_word_);
      pdo_channels_info_[index].ec_write(domain_address, control_word_);
    }
    return;
  }

  pdo_channels_info_[index].allow_ec_write = true;

  if (pdo_channels_info_[index].index == CiA402D_TPDO_POSITION) {
    if (mode_of_operation_display_ != MODE_CYCLIC_SYNC_POSITION &&
      mode_of_operation_display_ != MODE_PROFILED_POSITION &&
      mode_of_operation_display_ != MODE_INTERPOLATED_POSITION)
    {  // check if allowed to write position
      pdo_channels_info_[index].allow_ec_write = false;
    }
    pdo_channels_info_[index].default_value = last_position_;
  } else if (pdo_channels_info_[index].index == CiA402D_TPDO_VELOCITY) {
    if (mode_of_operation_display_ != MODE_CYCLIC_SYNC_VELOCITY &&
      mode_of_operation_display_ != MODE_PROFILED_VELOCITY)
    {  // check if allowed to write velocity
      pdo_channels_info_[index].allow_ec_write = false;
    }
  } else if (pdo_channels_info_[index].index == CiA402D_TPDO_EFFORT) {
    if (mode_of_operation_display_ != MODE_CYCLIC_SYNC_TORQUE &&
      mode_of_operation_display_ != MODE_PROFILED_TORQUE)
    {  // check if allowed to write effort
      pdo_channels_info_[index].allow_ec_write = false;
    }
  }

  pdo_channels_info_[index].ec_update(domain_address);

  // get mode_of_operation_display_
  if (pdo_channels_info_[index].index == CiA402D_RPDO_MODE_OF_OPERATION_DISPLAY) {
    mode_of_operation_display_ = pdo_channels_info_[index].last_value;
  }

  if (pdo_channels_info_[index].index == CiA402D_RPDO_POSITION) {
    last_position_ = pdo_channels_info_[index].last_value;
  }

  // Special case: StatusWord
  if (pdo_channels_info_[index].index == CiA402D_RPDO_STATUSWORD) {
    status_word_ = pdo_channels_info_[index].last_value;
  }


  // CHECK FOR STATE CHANGE
  if (index == all_channels_.size() - 1) {  // if last entry  in domain
    if (status_word_ != last_status_word_) {
      state_ = deviceState(status_word_);
      if (state_ != last_state_) {
        std::cout << "STATE: " << DEVICE_STATE_STR.at(state_)
                  << " with status word :" << status_word_ << std::endl;
      }
    }
    initialized_ = ((state_ == STATE_OPERATION_ENABLED) &&
      (last_state_ == STATE_OPERATION_ENABLED)) ? true : false;

    last_status_word_ = status_word_;
    last_state_ = state_;
    counter_++;
  }
}

const ec_sync_info_t * CiA402Drive::syncs()
{
  return syncs_.data();
}
size_t CiA402Drive::syncSize()
{
  return syncs_.size();
}
const ec_pdo_entry_info_t * CiA402Drive::channels()
{
  return all_channels_.data();
}
void CiA402Drive::domains(DomainMap & domains) const
{
  std::vector<unsigned int> v(all_channels_.size());
  std::iota(std::begin(v), std::end(v), 0);
  domains = {{0, v}};
}

void CiA402Drive::setup_syncs()
{
  syncs_.push_back({0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE});
  syncs_.push_back({1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE});
  syncs_.push_back({2, EC_DIR_OUTPUT, tpdos_.size(), tpdos_.data(), EC_WD_ENABLE});
  syncs_.push_back({3, EC_DIR_INPUT, rpdos_.size(), rpdos_.data(), EC_WD_DISABLE});
  syncs_.push_back({0xff});
}

bool CiA402Drive::setupSlave(
  std::unordered_map<std::string, std::string> slave_paramters,
  std::vector<double> * state_interface,
  std::vector<double> * command_interface)
{
  state_interface_ptr_ = state_interface;
  command_interface_ptr_ = command_interface;
  paramters_ = slave_paramters;

  if (paramters_.find("drive_config") != paramters_.end()) {
    if (!setup_from_config_file(paramters_["drive_config"])) {
      return false;
    }
  } else {
    std::cerr << "CiA402_Drive: failed to find 'drive_config' tag in URDF." << std::endl;
    return false;
  }

  if (paramters_.find("mode_of_operation") != paramters_.end()) {
    mode_of_operation_ = std::stod(paramters_["mode_of_operation"]);
  }

  if (paramters_.find("command_interface/reset_fault") != paramters_.end()) {
    fault_reset_command_interface_index_ = std::stoi(paramters_["command_interface/reset_fault"]);
  }

  setup_interface_mapping();
  setup_syncs();

  return true;
}

/** returns device state based upon the status_word */
DeviceState CiA402Drive::deviceState(uint16_t status_word)
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
uint16_t CiA402Drive::transition(DeviceState state, uint16_t control_word)
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

bool CiA402Drive::setup_from_config(YAML::Node drive_config)
{
  if (drive_config.size() != 0) {
    if (drive_config["vendor_id"]) {
      vendor_id_ = drive_config["vendor_id"].as<uint32_t>();
    } else {
      std::cerr << "CiA402_Drive: failed to load drive vendor ID." << std::endl;
      return false;
    }
    if (drive_config["product_id"]) {
      product_id_ = drive_config["product_id"].as<uint32_t>();
    } else {
      std::cerr << "CiA402_Drive: failed to load drive product ID." << std::endl;
      return false;
    }
    if (drive_config["assign_activate"]) {
      assign_activate_ = drive_config["assign_activate"].as<uint32_t>();
    }
    if (drive_config["auto_fault_reset"]) {
      auto_fault_reset_ = drive_config["auto_fault_reset"].as<bool>();
    }
    if (drive_config["sdo"]) {
      for (const auto & sdo : drive_config["sdo"]) {
        // todo read sdo
      }
    }

    auto channels_nbr = 0;

    if (drive_config["tpdo"]) {
      for (auto i = 0ul; i < drive_config["tpdo"].size(); i++) {
        channels_nbr += drive_config["tpdo"][i]["channels"].size();
      }
    }
    if (drive_config["rpdo"]) {
      for (auto i = 0ul; i < drive_config["rpdo"].size(); i++) {
        channels_nbr += drive_config["rpdo"][i]["channels"].size();
      }
    }

    all_channels_.reserve(channels_nbr);
    channels_nbr = 0;

    if (drive_config["tpdo"]) {
      for (auto i = 0ul; i < drive_config["tpdo"].size(); i++) {
        auto tpdo_channels_size = drive_config["tpdo"][i]["channels"].size();
        for (auto c = 0ul; c < tpdo_channels_size; c++) {
          EcPdoChannelManager channel_info;
          channel_info.pdo_type = TPDO;
          channel_info.index = drive_config["tpdo"][i]["channels"][c]["index"].as<uint16_t>();
          channel_info.sub_index =
            drive_config["tpdo"][i]["channels"][c]["sub_index"].as<uint8_t>();
          channel_info.data_type = drive_config["tpdo"][i]["channels"][c]["type"].as<std::string>();
          channel_info.interface_name =
            drive_config["tpdo"][i]["channels"][c]["command_interface"].as<std::string>();
          channel_info.default_value =
            drive_config["tpdo"][i]["channels"][c]["default"].as<double>();
          pdo_channels_info_.push_back(channel_info);
          all_channels_.push_back(channel_info.get_pdo_entry_info());
        }
        tpdos_.push_back(
          {
            drive_config["tpdo"][i]["index"].as<uint16_t>(),
            tpdo_channels_size,
            all_channels_.data() + channels_nbr
          }
        );
        channels_nbr += tpdo_channels_size;
      }
    }

    if (drive_config["rpdo"]) {
      for (auto i = 0ul; i < drive_config["rpdo"].size(); i++) {
        auto rpdo_channels_size = drive_config["rpdo"][i]["channels"].size();

        for (auto c = 0ul; c < rpdo_channels_size; c++) {
          EcPdoChannelManager channel_info;
          channel_info.pdo_type = RPDO;
          channel_info.index = drive_config["rpdo"][i]["channels"][c]["index"].as<uint16_t>();
          channel_info.sub_index =
            drive_config["rpdo"][i]["channels"][c]["sub_index"].as<uint8_t>();
          channel_info.data_type = drive_config["rpdo"][i]["channels"][c]["type"].as<std::string>();
          channel_info.interface_name =
            drive_config["rpdo"][i]["channels"][c]["state_interface"].as<std::string>();
          channel_info.default_value = std::numeric_limits<double>::quiet_NaN();
          pdo_channels_info_.push_back(channel_info);
          all_channels_.push_back(channel_info.get_pdo_entry_info());
        }
        rpdos_.push_back(
          {
            drive_config["rpdo"][i]["index"].as<uint16_t>(),
            rpdo_channels_size,
            all_channels_.data() + channels_nbr
          }
        );
        channels_nbr += rpdo_channels_size;
      }
    }

    return true;
  } else {
    std::cerr << "CiA402_Drive: failed to load drive configuration: empty configuration" <<
      std::endl;
    return false;
  }
}

bool CiA402Drive::setup_from_config_file(std::string config_file)
{
  // Read drive configuration from YAML file
  try {
    drive_config_ = YAML::LoadFile(config_file);
  } catch (const YAML::ParserException & ex) {
    std::cerr << "CiA402_Drive: failed to load drive configuration: " << ex.what() << std::endl;
    return false;
  } catch (const YAML::BadFile & ex) {
    std::cerr << "CiA402_Drive: failed to load drive configuration: " << ex.what() << std::endl;
    return false;
  }
  if (!setup_from_config(drive_config_)) {
    return false;
  }
  return true;
}

void CiA402Drive::setup_interface_mapping()
{
  for (auto & channel : pdo_channels_info_) {
    if (channel.pdo_type == RPDO) {
      if (paramters_.find("state_interface/" + channel.interface_name) != paramters_.end()) {
        channel.interface_index =
          std::stoi(paramters_["state_interface/" + channel.interface_name]);
      }
    }
    if (channel.pdo_type == TPDO) {
      if (paramters_.find("command_interface/" + channel.interface_name) != paramters_.end()) {
        channel.interface_index = std::stoi(
          paramters_["command_interface/" + channel.interface_name]);
      }
    }

    channel.setup_interface_ptrs(state_interface_ptr_, command_interface_ptr_);
  }
}

}  // namespace ethercat_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ethercat_plugins::CiA402Drive, ethercat_interface::EcSlave)
