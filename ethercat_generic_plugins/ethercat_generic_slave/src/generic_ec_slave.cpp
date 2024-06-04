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
#include "ethercat_interface/ec_pdo_single_interface_channel_manager.hpp"
#include "ethercat_interface/ec_pdo_group_interface_channel_manager.hpp"

namespace ethercat_generic_plugins
{

GenericEcSlave::GenericEcSlave()
: EcSlave(0, 0) {}
GenericEcSlave::~GenericEcSlave()
{
  for (size_t c = 0; c < pdo_channels_info_.size(); c++) {
    delete pdo_channels_info_[c];
  }
}
int GenericEcSlave::assign_activate_dc_sync() {return assign_activate_;}

void GenericEcSlave::processData(size_t entry_idx, uint8_t * domain_address)
{
  pdo_channels_info_[domain_map_[entry_idx]]->ec_update(domain_address);
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
    all_channels_skip_list_.reserve(channels_nbr);
    channels_nbr = 0;

    if (slave_config["rpdo"]) {
      for (auto i = 0ul; i < slave_config["rpdo"].size(); i++) {
        auto rpdo_channels_size = slave_config["rpdo"][i]["channels"].size();
        for (auto c = 0ul; c < rpdo_channels_size; c++) {
          ethercat_interface::EcPdoChannelManager * channel_info = nullptr;
          // Check if the channel is a special data area holding several in memory data
          if (slave_config["rpdo"][i]["channels"][c]["data_mapping"]) {
            channel_info = new ethercat_interface::EcPdoGroupInterfaceChannelManager;
          } else {
            channel_info = new ethercat_interface::EcPdoSingleInterfaceChannelManager;
          }

          channel_info->pdo_type = ethercat_interface::RPDO;
          channel_info->load_from_config(slave_config["rpdo"][i]["channels"][c]);
          pdo_channels_info_.push_back(channel_info);
          all_channels_.push_back(channel_info->get_pdo_entry_info());
          all_channels_skip_list_.push_back(channel_info->skip);
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
          ethercat_interface::EcPdoChannelManager * channel_info = nullptr;
          // Check if the channel is a special data area holding several in memory data
          if (slave_config["tpdo"][i]["channels"][c]["data_mapping"]) {
            channel_info = new ethercat_interface::EcPdoGroupInterfaceChannelManager;
          } else {
            channel_info = new ethercat_interface::EcPdoSingleInterfaceChannelManager;
          }
          channel_info->pdo_type = ethercat_interface::TPDO;
          channel_info->load_from_config(slave_config["tpdo"][i]["channels"][c]);
          pdo_channels_info_.push_back(channel_info);
          all_channels_.push_back(channel_info->get_pdo_entry_info());
          all_channels_skip_list_.push_back(channel_info->skip);
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
      if (all_channels_[i].index != 0x0000 && all_channels_skip_list_[i] != true) {
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
    std::cerr <<
      "GenericEcSlave: failed to load EtherCAT module configuration "
      "(YAML file is incorrect): " << ex.what() << std::endl;
    return false;
  } catch (const YAML::BadFile & ex) {
    std::cerr <<
      "GenericEcSlave: failed to load EtherCAT module configuration "
      "(file path is incorrect or file is damaged): " << ex.what()
              << std::endl;
    return false;
  }
  if (!setup_from_config(slave_config_)) {
    return false;
  }
  return true;
}

void GenericEcSlave::setup_interface_mapping()
{
  for (auto & channel_ptr : pdo_channels_info_) {
    auto & channel = *channel_ptr;
    for (size_t i = 0; i < channel.number_of_interfaces(); ++i) {
      if (channel.has_state_interface_name(i) ) {
        std::string interface = "state_interface/" + channel.interface_name(i);
        if (parameters_.find(interface) != parameters_.end()) {
          const size_t idx = std::stoi(parameters_[interface]);
          channel.set_state_interface_index(channel.interface_name(i), idx);
        }
      } else if (channel.has_command_interface_name(i) ) {
        std::string interface = "command_interface/" + channel.interface_name(i);
        if (channel.pdo_type == ethercat_interface::RPDO) {
          std::string interface = "command_interface/" + channel.interface_name(i);
          if (parameters_.find(interface) != parameters_.end()) {
            const size_t idx = std::stoi(parameters_[interface]);
            channel.set_command_interface_index(channel.interface_name(i), idx);
          }
        } else {
          throw std::runtime_error(
                  std::string("GenericEcSlave: command interface (") +
                  "index: " + channel.index_hex_str() + ", " +
                  "sub_index: " + channel.sub_index_hex_str() + ", " +
                  "name: " + interface +
                  ") is not allowed for TPDO channels");
        }
      }
    }

    channel.setup_interface_ptrs(state_interface_ptr_, command_interface_ptr_);
  }
}

}  // namespace ethercat_generic_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ethercat_generic_plugins::GenericEcSlave, ethercat_interface::EcSlave)
