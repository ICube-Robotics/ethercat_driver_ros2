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

class Advantech_AMAX5056 : public ethercat_interface::EcSlave
{
public:
  Advantech_AMAX5056()
  : EcSlave(0x000013FE, 0x00035056)
  {
    std::cerr << "The Advantech_AMAX5056 plugin is depreciated and will be removed in the future."
              << "Use the GenericEcSlave plugin instead." << std::endl;
  }
  virtual ~Advantech_AMAX5056() {}
  virtual void processData(size_t index, uint8_t * domain_address)
  {
    uint8_t digital_out_;
    digital_out_ = 0;
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
    digital_out_ += ( write_data_[0] << 0 );  // bit 0
    digital_out_ += ( write_data_[1] << 1 );  // bit 1
    digital_out_ += ( write_data_[2] << 2 );  // bit 2
    digital_out_ += ( write_data_[3] << 3 );  // bit 3
    digital_out_ += ( write_data_[4] << 4 );  // bit 4
    digital_out_ += ( write_data_[5] << 5 );  // bit 5
    digital_out_ += ( write_data_[6] << 6 );  // bit 6
    digital_out_ += ( write_data_[7] << 7 );  // bit 7

    EC_WRITE_U8(domain_address, digital_out_);
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
      if (paramters_.find("do." + std::to_string(index)) != paramters_.end()) {
        if (paramters_.find(
            "command_interface/" + paramters_["do." + std::to_string(index)]) != paramters_.end())
        {
          cii_do_[index] = std::stoi(
            paramters_["command_interface/" + paramters_["do." + std::to_string(index)]]);
        }
        if (paramters_.find(
            "state_interface/" + paramters_["do." + std::to_string(index)]) != paramters_.end())
        {
          sii_do_[index] = std::stoi(
            paramters_["state_interface/" + paramters_["do." + std::to_string(index)]]);
        }
      }
    }
    return true;
  }

private:
  int cii_do_[8] = {-1, -1, -1, -1, -1, -1, -1, -1};
  int sii_do_[8] = {-1, -1, -1, -1, -1, -1, -1, -1};
  // digital write values
  bool write_data_[8] = {false, false, false, false, false, false, false, false};
  ec_pdo_entry_info_t channels_[8] = {
    {0x3101, 0x01, 1},  /* Output */
    {0x3101, 0x02, 1},  /* Output */
    {0x3101, 0x03, 1},  /* Output */
    {0x3101, 0x04, 1},  /* Output */
    {0x3101, 0x05, 1},  /* Output */
    {0x3101, 0x06, 1},  /* Output */
    {0x3101, 0x07, 1},  /* Output */
    {0x3101, 0x08, 1},  /* Output */
  };
  ec_pdo_info_t pdos_[1] = {
    {0x1600, 1, channels_ + 0},  /* Channel 1 */
  };
  ec_sync_info_t syncs_[2] = {
    {0, EC_DIR_OUTPUT, 1, pdos_ + 0, EC_WD_ENABLE},
    {0xff}
  };
  DomainMap domains_ = {
    {0, {0}}
  };
};
}  // namespace ethercat_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ethercat_plugins::Advantech_AMAX5056, ethercat_interface::EcSlave)
