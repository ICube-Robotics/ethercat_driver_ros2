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

namespace ethercat_plugins
{

class Omron_NX_ECC201_NX_ID5442 : public ethercat_interface::EcSlave
{
public:
  Omron_NX_ECC201_NX_ID5442()
  : EcSlave(0x00000083, 0x00000083)
  {
    std::cerr <<
      "The Omron_NX_ECC201_NX_ID5442 plugin is depreciated and will be removed in the future."
              << "Use the GenericEcSlave plugin instead." << std::endl;
  }
  virtual ~Omron_NX_ECC201_NX_ID5442() {}
  virtual void processData(size_t /*index*/, uint8_t * domain_address)
  {
    uint16_t data = 0;
    data = EC_READ_U16(domain_address);
    bool digital_inputs_[16];
    digital_inputs_[0] = ((data & 0b0000000000000001) != 0);    // bit 0
    digital_inputs_[1] = ((data & 0b0000000000000010) != 0);    // bit 1
    digital_inputs_[2] = ((data & 0b0000000000000100) != 0);    // bit 2
    digital_inputs_[3] = ((data & 0b0000000000001000) != 0);    // bit 3
    digital_inputs_[4] = ((data & 0b0000000000010000) != 0);    // bit 4
    digital_inputs_[5] = ((data & 0b0000000000100000) != 0);    // bit 5
    digital_inputs_[6] = ((data & 0b0000000001000000) != 0);    // bit 6
    digital_inputs_[7] = ((data & 0b0000000010000000) != 0);    // bit 7
    digital_inputs_[8] = ((data & 0b0000000100000000) != 0);    // bit 8
    digital_inputs_[9] = ((data & 0b0000001000000000) != 0);    // bit 9
    digital_inputs_[10] = ((data & 0b0000010000000000) != 0);    // bit 10
    digital_inputs_[11] = ((data & 0b0000100000000000) != 0);    // bit 11
    digital_inputs_[12] = ((data & 0b0001000000000000) != 0);    // bit 12
    digital_inputs_[13] = ((data & 0b0010000000000000) != 0);    // bit 13
    digital_inputs_[14] = ((data & 0b0100000000000000) != 0);    // bit 14
    digital_inputs_[15] = ((data & 0b1000000000000000) != 0);    // bit 15

    for (auto i = 0ul; i < 16; i++) {
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

    for (auto index = 0; index < 16; index++) {
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
  std::vector<int> sii_di_{std::vector<int>(16, -1)};
  // digital write values
  std::vector<bool> write_data_{std::vector<bool>(16, false)};

  ec_pdo_entry_info_t channels_[6] = {
    {0x7022, 0x01, 16},
    {0x3003, 0x04, 128},
    {0x3006, 0x04, 128},
    {0x2002, 0x01, 8},
    {0x0000, 0x00, 8},  /* Gap */
    {0x6002, 0x01, 16},
  };
  ec_pdo_info_t pdos_[5] = {
    {0x1604, 1, channels_ + 0},
    {0x1bf8, 2, channels_ + 1},
    {0x1bff, 1, channels_ + 3},
    {0x1bf4, 1, channels_ + 4},
    {0x1a00, 1, channels_ + 5},
  };
  ec_sync_info_t syncs_[5] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, pdos_ + 0, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 4, pdos_ + 1, EC_WD_DISABLE},
    {0xff}
  };
  DomainMap domains_ = {
    {0, {5}}
  };
};

}  // namespace ethercat_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ethercat_plugins::Omron_NX_ECC201_NX_ID5442, ethercat_interface::EcSlave)
