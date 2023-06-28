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

namespace ethercat_plugins
{

class Advantech_AMAX5051 : public ethercat_interface::EcSlave
{
public:
  Advantech_AMAX5051()
  : EcSlave(0x000013FE, 0x00035051)
  {
    std::cerr << "The Advantech_AMAX5051 plugin is depreciated and will be removed in the future."
              << "Use the GenericEcSlave plugin instead." << std::endl;
  }
  virtual ~Advantech_AMAX5051() {}
  virtual void processData(size_t index, uint8_t * domain_address)
  {
    uint8_t data = 0;
    data = EC_READ_U8(domain_address);
    digital_inputs_[0] = ((data & 0b00000001) != 0);    // bit 0
    digital_inputs_[1] = ((data & 0b00000010) != 0);    // bit 1
    digital_inputs_[2] = ((data & 0b00000100) != 0);    // bit 2
    digital_inputs_[3] = ((data & 0b00001000) != 0);    // bit 3
    digital_inputs_[4] = ((data & 0b00010000) != 0);    // bit 4
    digital_inputs_[5] = ((data & 0b00100000) != 0);    // bit 5
    digital_inputs_[6] = ((data & 0b01000000) != 0);    // bit 6
    digital_inputs_[7] = ((data & 0b10000000) != 0);    // bit 7
    for (auto i = 0ul; i < 8; i++) {
      if (sii_di_[i] >= 0) {
        state_interface_ptr_->at(sii_di_[i]) = digital_inputs_[i];
      }
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

    for (auto index = 0ul; index < 8; index++) {
      if (paramters_.find("di." + std::to_string(index)) != paramters_.end()) {
        if (paramters_.find(
            "state_interface/" + paramters_["di." + std::to_string(index)]) != paramters_.end())
        {
          sii_di_[index] = std::stoi(
            paramters_["state_interface/" + paramters_["di." + std::to_string(index)]]);
        }
      }
    }
    return true;
  }

private:
  int sii_di_[8] = {-1, -1, -1, -1, -1, -1, -1, -1};
  // digital write values
  bool write_data_[8] = {false, false, false, false, false, false, false, false};
  bool digital_inputs_[8];

  ec_pdo_entry_info_t channels_[1] = {
    {0x3001, 0x00, 1},  /* Input */
  };
  ec_pdo_info_t pdos_[1] = {
    {0x1a00, 1, channels_ + 0},  /* Channel 1 */
  };
  ec_sync_info_t syncs_[2] = {
    {0, EC_DIR_INPUT, 1, pdos_ + 0, EC_WD_DISABLE},
    {0xff}
  };
  DomainMap domains_ = {
    {0, {0}}
  };
};
}  // namespace ethercat_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ethercat_plugins::Advantech_AMAX5051, ethercat_interface::EcSlave)
