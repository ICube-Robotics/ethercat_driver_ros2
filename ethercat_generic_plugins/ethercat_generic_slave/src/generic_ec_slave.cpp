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
  } else {
    return 0;
  }
}

namespace ethercat_generic_plugins
{

GenericEcSlave::GenericEcSlave()
: EcSlave() {}
GenericEcSlave::~GenericEcSlave() {}

int GenericEcSlave::process_data(size_t index, uint8_t * domain_address)
{
  pdo_channels_info_[index].ec_update(domain_address);
  return 0;
}

bool GenericEcSlave::setup_slave(
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

  return true;
}

bool GenericEcSlave::setup_from_config(YAML::Node slave_config)
{
  if (slave_config.size() != 0) {
    // configure slave vendor id
    if (slave_config["vendor_id"]) {
      vendor_id_ = slave_config["vendor_id"].as<uint32_t>();
    } else {
      std::cerr << "GenericEcSlave: failed to load drive vendor ID." << std::endl;
      return false;
    }
    // configure slave product id
    if (slave_config["product_id"]) {
      product_id_ = slave_config["product_id"].as<uint32_t>();
    } else {
      std::cerr << "GenericEcSlave: failed to load drive product ID." << std::endl;
      return false;
    }
    // configure sc sync
    if (slave_config["assign_activate"]) {
      assign_activate_ = slave_config["assign_activate"].as<uint32_t>();
    }
    // configure sync manager
    if (slave_config["sm"]) {
      for (const auto & sm : slave_config["sm"]) {
        ethercat_interface::SMConfig config;
        if (config.load_from_config(sm)) {
          sm_config_.push_back(config);
        }
      }
    }
    // configure sdo
    if (slave_config["sdo"]) {
      for (const auto & sdo : slave_config["sdo"]) {
        ethercat_interface::SdoConfigEntry config;
        if (config.load_from_config(sdo)) {
          sdo_config_.push_back(config);
        }
      }
    }
    // configure rpdo
    if (slave_config["rpdo"]) {
      for (auto i = 0ul; i < slave_config["rpdo"].size(); i++) {
        auto rpdo_channels_size = slave_config["rpdo"][i]["channels"].size();
        ethercat_interface::pdoc_vec_t rpdo_channels_info;
        for (auto c = 0ul; c < rpdo_channels_size; c++) {
          ethercat_interface::EcPdoChannelManager channel_info;
          channel_info.pdo_type = ethercat_interface::RPDO;
          channel_info.load_from_config(slave_config["rpdo"][i]["channels"][c]);
          rpdo_channels_info.push_back(channel_info);
          pdo_channels_info_.push_back(channel_info);
        }
        pdo_config_map_.emplace(
          slave_config["rpdo"][i]["index"].as<uint16_t>(),
          rpdo_channels_info
        );
      }
    }
    // configure tpdo
    if (slave_config["tpdo"]) {
      for (auto i = 0ul; i < slave_config["tpdo"].size(); i++) {
        auto tpdo_channels_size = slave_config["tpdo"][i]["channels"].size();
        ethercat_interface::pdoc_vec_t tpdo_channels_info;
        for (auto c = 0ul; c < tpdo_channels_size; c++) {
          ethercat_interface::EcPdoChannelManager channel_info;
          channel_info.pdo_type = ethercat_interface::TPDO;
          channel_info.mapping_index = slave_config["tpdo"][i]["index"].as<uint16_t>(),
          channel_info.load_from_config(slave_config["tpdo"][i]["channels"][c]);
          tpdo_channels_info.push_back(channel_info);
          pdo_channels_info_.push_back(channel_info);
        }
        pdo_config_map_.emplace(
          slave_config["tpdo"][i]["index"].as<uint16_t>(),
          tpdo_channels_info
        );
      }
    }
  } else {
    std::cerr << "GenericEcSlave: failed to load slave configuration: empty configuration" <<
      std::endl;
    return false;
  }

  return true;
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

YAML::Node GenericEcSlave::get_slave_config()
{
  return slave_config_;
}
}  // namespace ethercat_generic_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ethercat_generic_plugins::GenericEcSlave, ethercat_interface::EcSlave)
