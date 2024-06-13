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

#include "ethercat_generic_plugins/generic_ec_slave.hpp"


size_t type2bytes(std::string type)
{
  if (type == "int8" || type == "uint8") {
    return 1;
  } else if (type == "int16" || type == "uint16") {
    return 2;
  } else if (type == "int32" || type == "uint32") {
    return 4;
  } else if (type == "int64" || type == "uint64") {
    return 8;
  }
  return 0;
}

namespace ethercat_generic_plugins
{

GenericEcSlave::GenericEcSlave()
: EcSlave(0, 0) {}
GenericEcSlave::~GenericEcSlave() {}
int GenericEcSlave::assign_activate_dc_sync() {return assign_activate_;}

void GenericEcSlave::processData(size_t index, uint8_t * domain_address)
{
  pdo_channels_info_[domain_map_[index]].ec_update(domain_address);
}

const ec_sync_info_t * GenericEcSlave::syncs()
{
  return syncs_.data();
}
size_t GenericEcSlave::syncSize()
{
  return syncs_.size();
}
const ec_pdo_entry_info_t * GenericEcSlave::channels()
{
  return all_channels_.data();
}
void GenericEcSlave::domains(DomainMap & domains) const
{
  domains = {{0, domain_map_}};
}

void GenericEcSlave::setup_syncs()
{
  if (sm_configs_.size() == 0) {
    syncs_.push_back({0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE});
    syncs_.push_back({1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE});
    syncs_.push_back(
      {2, EC_DIR_OUTPUT, (unsigned int)(rpdos_.size()), rpdos_.data(),
        EC_WD_ENABLE});
    syncs_.push_back(
      {3, EC_DIR_INPUT, (unsigned int)(tpdos_.size()), tpdos_.data(),
        EC_WD_DISABLE});
  } else {
    for (auto & sm : sm_configs_) {
      if (sm.pdo_name == "null") {
        syncs_.push_back({sm.index, sm.type, 0, NULL, sm.watchdog});
      } else if (sm.pdo_name == "rpdo") {
        syncs_.push_back(
          {sm.index, sm.type, (unsigned int)(rpdos_.size()),
            rpdos_.data(), sm.watchdog});
      } else if (sm.pdo_name == "tpdo") {
        syncs_.push_back(
          {sm.index, sm.type, (unsigned int)(tpdos_.size()),
            tpdos_.data(), sm.watchdog});
      }
    }
  }
  syncs_.push_back({0xff, EC_DIR_INVALID, 0, nullptr, EC_WD_DISABLE});
}

bool GenericEcSlave::setupSlave(
  std::unordered_map<std::string, std::string> slave_paramters,
  std::vector<double> * state_interface,
  std::vector<double> * command_interface)
{
  state_interface_ptr_ = state_interface;
  command_interface_ptr_ = command_interface;
  paramters_ = slave_paramters;

  if (paramters_.find("slave_config") != paramters_.end()) {
    if (!setup_from_config_file(paramters_["slave_config"])) {
      return false;
    }
  } else {
    std::cerr << "GenericEcSlave: failed to find 'slave_config' tag in URDF." << std::endl;
    return false;
  }

  setup_interface_mapping();
  setup_syncs();

  return true;
}

bool GenericEcSlave::setup_from_config(YAML::Node slave_config)
{
  if (slave_config.size() != 0) {
    if (slave_config["vendor_id"]) {
      vendor_id_ = slave_config["vendor_id"].as<uint32_t>();
    } else {
      std::cerr << "GenericEcSlave: failed to load drive vendor ID." << std::endl;
      return false;
    }
    if (slave_config["product_id"]) {
      product_id_ = slave_config["product_id"].as<uint32_t>();
    } else {
      std::cerr << "GenericEcSlave: failed to load drive product ID." << std::endl;
      return false;
    }
    if (slave_config["assign_activate"]) {
      assign_activate_ = slave_config["assign_activate"].as<uint32_t>();
    }

    if (slave_config["sm"]) {
      for (const auto & sm : slave_config["sm"]) {
        ethercat_interface::SMConfig config;
        if (config.load_from_config(sm)) {
          sm_configs_.push_back(config);
        }
      }
    }

    if (slave_config["sdo"]) {
      for (const auto & sdo : slave_config["sdo"]) {
        ethercat_interface::SdoConfigEntry config;
        if (config.load_from_config(sdo)) {
          sdo_config.push_back(config);
        }
      }
    }

    auto channels_nbr = 0;

    if (slave_config["rpdo"]) {
      for (auto i = 0ul; i < slave_config["rpdo"].size(); i++) {
        channels_nbr += slave_config["rpdo"][i]["channels"].size();
      }
    }
    if (slave_config["tpdo"]) {
      for (auto i = 0ul; i < slave_config["tpdo"].size(); i++) {
        channels_nbr += slave_config["tpdo"][i]["channels"].size();
      }
    }

    all_channels_.reserve(channels_nbr);
    channels_nbr = 0;

    if (slave_config["rpdo"]) {
      for (auto i = 0ul; i < slave_config["rpdo"].size(); i++) {
        auto rpdo_channels_size = slave_config["rpdo"][i]["channels"].size();
        for (auto c = 0ul; c < rpdo_channels_size; c++) {
          ethercat_interface::EcPdoChannelManager channel_info;
          channel_info.pdo_type = ethercat_interface::RPDO;
          channel_info.load_from_config(slave_config["rpdo"][i]["channels"][c]);
          pdo_channels_info_.push_back(channel_info);
          all_channels_.push_back(channel_info.get_pdo_entry_info());
        }
        rpdos_.push_back(
          {
            slave_config["rpdo"][i]["index"].as<uint16_t>(),
            (unsigned int)(rpdo_channels_size),
            all_channels_.data() + channels_nbr
          }
        );
        channels_nbr += rpdo_channels_size;
      }
    }

    if (slave_config["tpdo"]) {
      for (auto i = 0ul; i < slave_config["tpdo"].size(); i++) {
        auto tpdo_channels_size = slave_config["tpdo"][i]["channels"].size();

        for (auto c = 0ul; c < tpdo_channels_size; c++) {
          ethercat_interface::EcPdoChannelManager channel_info;
          channel_info.pdo_type = ethercat_interface::TPDO;
          channel_info.load_from_config(slave_config["tpdo"][i]["channels"][c]);
          pdo_channels_info_.push_back(channel_info);
          all_channels_.push_back(channel_info.get_pdo_entry_info());
        }
        tpdos_.push_back(
          {
            slave_config["tpdo"][i]["index"].as<uint16_t>(),
            (unsigned int)(tpdo_channels_size),
            all_channels_.data() + channels_nbr
          }
        );
        channels_nbr += tpdo_channels_size;
      }
    }

    // Remove gaps from domain mapping
    for (auto i = 0ul; i < all_channels_.size(); i++) {
      if (all_channels_[i].index != 0x0000) {
        domain_map_.push_back(i);
      }
    }

    return true;
  } else {
    std::cerr << "GenericEcSlave: failed to load slave configuration: empty configuration" <<
      std::endl;
    return false;
  }
}

bool GenericEcSlave::setup_from_config_file(std::string config_file)
{
  // Read drive configuration from YAML file
  try {
    slave_config_ = YAML::LoadFile(config_file);
  } catch (const YAML::ParserException & ex) {
    std::cerr << "GenericEcSlave: failed to load drive configuration: " << ex.what() << std::endl;
    return false;
  } catch (const YAML::BadFile & ex) {
    std::cerr << "GenericEcSlave: failed to load drive configuration: " << ex.what() << std::endl;
    return false;
  }
  if (!setup_from_config(slave_config_)) {
    return false;
  }
  return true;
}

void GenericEcSlave::setup_interface_mapping()
{
  for (auto & channel : pdo_channels_info_) {
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

}  // namespace ethercat_generic_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ethercat_generic_plugins::GenericEcSlave, ethercat_interface::EcSlave)
