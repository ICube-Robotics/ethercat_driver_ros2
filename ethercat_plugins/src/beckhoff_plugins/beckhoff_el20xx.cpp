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

#include <iostream>
#include "ethercat_interface/ec_slave.hpp"
#include "ethercat_plugins/commondefs.hpp"

namespace ethercat_plugins
{

class Beckhoff_EL2008 : public ethercat_interface::EcSlave
{
public:
  Beckhoff_EL2008()
  : EcSlave(0x00000002, 0x07d83052) {}
  virtual ~Beckhoff_EL2008() {}
  virtual void processData(size_t index, uint8_t * domain_address)
  {
    uint8_t digital_out_;
    digital_out_ = 0;

    bool isRequested[8] = {false};
    for (auto i = 0ul; i < 8; i++) {
      isRequested[i] = paramters_.find("do." + std::to_string(i + 1)) != paramters_.end();
      if (isRequested[i]) {
        int command_iface =
          std::stoi(paramters_["command_interface/" + paramters_["do." + std::to_string(i + 1)]]);
        int state_iface =
          std::stoi(paramters_["state_interface/" + paramters_["do." + std::to_string(i + 1)]]);

        if (command_interface_ptr_->at(command_iface) ==
          command_interface_ptr_->at(command_iface))
        {
          if (command_interface_ptr_->at(command_iface)) {
            write_data_[i] = true;
          } else {
            write_data_[i] = false;
          }
          state_interface_ptr_->at(state_iface) = command_interface_ptr_->at(command_iface);
        }
      }
    }

    // bit masking to get individual input values
    digital_out_ += (write_data_[0] << 0);  // bit 0
    digital_out_ += (write_data_[1] << 1);  // bit 1
    digital_out_ += (write_data_[2] << 2);  // bit 2
    digital_out_ += (write_data_[3] << 3);  // bit 3
    digital_out_ += (write_data_[4] << 4);  // bit 4
    digital_out_ += (write_data_[5] << 5);  // bit 5
    digital_out_ += (write_data_[6] << 6);  // bit 6
    digital_out_ += (write_data_[7] << 7);  // bit 7

    EC_WRITE_U8(domain_address, digital_out_);
  }
  virtual const ec_sync_info_t * syncs() {return &syncs_[0];}
  virtual size_t syncSize() {return sizeof(syncs_) / sizeof(ec_sync_info_t);}
  virtual const ec_pdo_entry_info_t * channels() {return channels_;}
  virtual void domains(DomainMap & domains) const {domains = domains_;}
  // digital write values
  bool write_data_[8] = {false};

private:
  ec_pdo_entry_info_t channels_[8] = {
    {0x7000, 0x01, 1},   /* Output */
    {0x7010, 0x01, 1},   /* Output */
    {0x7020, 0x01, 1},   /* Output */
    {0x7030, 0x01, 1},   /* Output */
    {0x7040, 0x01, 1},   /* Output */
    {0x7050, 0x01, 1},   /* Output */
    {0x7060, 0x01, 1},   /* Output */
    {0x7070, 0x01, 1},   /* Output */
  };
  ec_pdo_info_t pdos_[8] = {
    {0x1600, 1, channels_ + 0},   /* Channel 1 */
    {0x1601, 1, channels_ + 1},   /* Channel 2 */
    {0x1602, 1, channels_ + 2},   /* Channel 3 */
    {0x1603, 1, channels_ + 3},   /* Channel 4 */
    {0x1604, 1, channels_ + 4},   /* Channel 5 */
    {0x1605, 1, channels_ + 5},   /* Channel 6 */
    {0x1606, 1, channels_ + 6},   /* Channel 7 */
    {0x1607, 1, channels_ + 7},   /* Channel 8 */
  };
  ec_sync_info_t syncs_[2] = {{0, EC_DIR_OUTPUT, 8, pdos_ + 0, EC_WD_ENABLE}, {0xff}};
  DomainMap domains_ = {{0, {0}}};
};
// --------------------------------------------------------------------------------------
class Beckhoff_EL2088 : public ethercat_interface::EcSlave
{
public:
  Beckhoff_EL2088()
  : EcSlave(0x00000002, 0x08283052) {}
  virtual ~Beckhoff_EL2088() {}
  virtual void processData(size_t index, uint8_t * domain_address)
  {
    uint8_t digital_out_;
    digital_out_ = 0;

    bool isRequested[8] = {false};
    for (auto i = 0ul; i < 8; i++) {
      isRequested[i] = paramters_.find("do." + std::to_string(i + 1)) != paramters_.end();
      if (isRequested[i]) {
        int command_iface =
          std::stoi(paramters_["command_interface/" + paramters_["do." + std::to_string(i + 1)]]);
        int state_iface =
          std::stoi(paramters_["state_interface/" + paramters_["do." + std::to_string(i + 1)]]);

        if (command_interface_ptr_->at(command_iface) ==
          command_interface_ptr_->at(command_iface))
        {
          if (command_interface_ptr_->at(command_iface)) {
            write_data_[i] = true;
          } else {
            write_data_[i] = false;
          }
          state_interface_ptr_->at(state_iface) = command_interface_ptr_->at(command_iface);
        }
      }
    }

    // bit masking to get individual input values
    digital_out_ += (write_data_[0] << 0);  // bit 0
    digital_out_ += (write_data_[1] << 1);  // bit 1
    digital_out_ += (write_data_[2] << 2);  // bit 2
    digital_out_ += (write_data_[3] << 3);  // bit 3
    digital_out_ += (write_data_[4] << 4);  // bit 4
    digital_out_ += (write_data_[5] << 5);  // bit 5
    digital_out_ += (write_data_[6] << 6);  // bit 6
    digital_out_ += (write_data_[7] << 7);  // bit 7

    EC_WRITE_U8(domain_address, digital_out_);
  }
  virtual const ec_sync_info_t * syncs() {return &syncs_[0];}
  virtual size_t syncSize() {return sizeof(syncs_) / sizeof(ec_sync_info_t);}
  virtual const ec_pdo_entry_info_t * channels() {return channels_;}
  virtual void domains(DomainMap & domains) const {domains = domains_;}
  // digital write values
  bool write_data_[8] = {false};

private:
  ec_pdo_entry_info_t channels_[8] = {
    {0x7000, 0x01, 1},   /* Output */
    {0x7010, 0x01, 1},   /* Output */
    {0x7020, 0x01, 1},   /* Output */
    {0x7030, 0x01, 1},   /* Output */
    {0x7040, 0x01, 1},   /* Output */
    {0x7050, 0x01, 1},   /* Output */
    {0x7060, 0x01, 1},   /* Output */
    {0x7070, 0x01, 1},   /* Output */
  };
  ec_pdo_info_t pdos_[8] = {
    {0x1600, 1, channels_ + 0},   /* Channel 1 */
    {0x1601, 1, channels_ + 1},   /* Channel 2 */
    {0x1602, 1, channels_ + 2},   /* Channel 3 */
    {0x1603, 1, channels_ + 3},   /* Channel 4 */
    {0x1604, 1, channels_ + 4},   /* Channel 5 */
    {0x1605, 1, channels_ + 5},   /* Channel 6 */
    {0x1606, 1, channels_ + 6},   /* Channel 7 */
    {0x1607, 1, channels_ + 7},   /* Channel 8 */
  };
  ec_sync_info_t syncs_[2] = {{0, EC_DIR_OUTPUT, 8, pdos_ + 0, EC_WD_ENABLE}, {0xff}};
  DomainMap domains_ = {{0, {0}}};
};

}  // namespace ethercat_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ethercat_plugins::Beckhoff_EL2008, ethercat_interface::EcSlave)
PLUGINLIB_EXPORT_CLASS(ethercat_plugins::Beckhoff_EL2088, ethercat_interface::EcSlave)
