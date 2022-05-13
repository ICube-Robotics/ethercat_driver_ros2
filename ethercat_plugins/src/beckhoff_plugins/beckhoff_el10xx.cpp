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
#include "ethercat_plugins/commondefs.hpp"
#include <iostream>

namespace ethercat_plugins
{

class Beckhoff_EL1008 : public ethercat_interface::EcSlave
{
public:
    Beckhoff_EL1008() : EcSlave(0x00000002, 0x03f03052) {}
    virtual ~Beckhoff_EL1008() {}
    virtual void processData(size_t index, uint8_t* domain_address){
	    uint8_t data = 0;
        data = EC_READ_U8(domain_address);
        bool digital_inputs_[8];
        digital_inputs_[0] = ((data & 0b00000001) != 0); // bit 0
        digital_inputs_[1] = ((data & 0b00000010) != 0); // bit 1
        digital_inputs_[2] = ((data & 0b00000100) != 0); // bit 2
        digital_inputs_[3] = ((data & 0b00001000) != 0); // bit 3
        digital_inputs_[4] = ((data & 0b00010000) != 0); // bit 4
        digital_inputs_[5] = ((data & 0b00100000) != 0); // bit 5
        digital_inputs_[6] = ((data & 0b01000000) != 0); // bit 6
        digital_inputs_[7] = ((data & 0b10000000) != 0); // bit 7
        for(auto i=0ul;i<8;i++){
            bool isRequested = paramters_.find("di."+std::to_string(i+1))!= paramters_.end();
            if(isRequested){
                state_interface_ptr_->at(std::stoi(paramters_["state_interface/"+paramters_["di."+std::to_string(i+1)]])) = digital_inputs_[i];
            }
        }
    }
    virtual const ec_sync_info_t* syncs() { return &syncs_[0]; }
    virtual size_t syncSize() {
        return sizeof(syncs_)/sizeof(ec_sync_info_t);
    }
    virtual const ec_pdo_entry_info_t* channels() {
        return channels_;
    }
    virtual void domains(DomainMap& domains) const {
        domains = domains_;
    }
    // digital write values
    bool write_data_[8] = {false};
private:
    ec_pdo_entry_info_t channels_[8] = {
        {0x6000, 0x01, 1}, /* Input */
        {0x6010, 0x01, 1}, /* Input */
        {0x6020, 0x01, 1}, /* Input */
        {0x6030, 0x01, 1}, /* Input */
        {0x6040, 0x01, 1}, /* Input */
        {0x6050, 0x01, 1}, /* Input */
        {0x6060, 0x01, 1}, /* Input */
        {0x6070, 0x01, 1}, /* Input */
    };
    ec_pdo_info_t pdos_[8] = {
        {0x1a00, 1, channels_ + 0}, /* Channel 1 */
        {0x1a01, 1, channels_ + 1}, /* Channel 2 */
        {0x1a02, 1, channels_ + 2}, /* Channel 3 */
        {0x1a03, 1, channels_ + 3}, /* Channel 4 */
        {0x1a04, 1, channels_ + 4}, /* Channel 5 */
        {0x1a05, 1, channels_ + 5}, /* Channel 6 */
        {0x1a06, 1, channels_ + 6}, /* Channel 7 */
        {0x1a07, 1, channels_ + 7}, /* Channel 8 */
    };
    ec_sync_info_t syncs_[2] = {
        {0, EC_DIR_INPUT, 8, pdos_ + 0, EC_WD_DISABLE},
        {0xff}
    };
    DomainMap domains_ = {
        {0, {0} }
    };
};

// --------------------------------------------------------------------------------------
class Beckhoff_EL1018 : public ethercat_interface::EcSlave
{
public:
    Beckhoff_EL1018() : EcSlave(0x00000002, 0x03fa3052) {}
    virtual ~Beckhoff_EL1018() {}
    virtual void processData(size_t index, uint8_t* domain_address){
	    uint8_t data = 0;
        data = EC_READ_U8(domain_address);
        bool digital_inputs_[8];
        digital_inputs_[0] = ((data & 0b00000001) != 0); // bit 0
        digital_inputs_[1] = ((data & 0b00000010) != 0); // bit 1
        digital_inputs_[2] = ((data & 0b00000100) != 0); // bit 2
        digital_inputs_[3] = ((data & 0b00001000) != 0); // bit 3
        digital_inputs_[4] = ((data & 0b00010000) != 0); // bit 4
        digital_inputs_[5] = ((data & 0b00100000) != 0); // bit 5
        digital_inputs_[6] = ((data & 0b01000000) != 0); // bit 6
        digital_inputs_[7] = ((data & 0b10000000) != 0); // bit 7
        for(auto i=0ul;i<8;i++){
            bool isRequested = paramters_.find("di."+std::to_string(i+1))!= paramters_.end();
            if(isRequested){
                state_interface_ptr_->at(std::stoi(paramters_["state_interface/"+paramters_["di."+std::to_string(i+1)]])) = digital_inputs_[i];
            }
        }
    }
    virtual const ec_sync_info_t* syncs() { return &syncs_[0]; }
    virtual size_t syncSize() {
        return sizeof(syncs_)/sizeof(ec_sync_info_t);
    }
    virtual const ec_pdo_entry_info_t* channels() {
        return channels_;
    }
    virtual void domains(DomainMap& domains) const {
        domains = domains_;
    }
    // digital write values
    bool write_data_[8] = {false};
private:
    ec_pdo_entry_info_t channels_[8] = {
        {0x6000, 0x01, 1}, /* Input */
        {0x6010, 0x01, 1}, /* Input */
        {0x6020, 0x01, 1}, /* Input */
        {0x6030, 0x01, 1}, /* Input */
        {0x6040, 0x01, 1}, /* Input */
        {0x6050, 0x01, 1}, /* Input */
        {0x6060, 0x01, 1}, /* Input */
        {0x6070, 0x01, 1}, /* Input */
    };
    ec_pdo_info_t pdos_[8] = {
        {0x1a00, 1, channels_ + 0}, /* Channel 1 */
        {0x1a01, 1, channels_ + 1}, /* Channel 2 */
        {0x1a02, 1, channels_ + 2}, /* Channel 3 */
        {0x1a03, 1, channels_ + 3}, /* Channel 4 */
        {0x1a04, 1, channels_ + 4}, /* Channel 5 */
        {0x1a05, 1, channels_ + 5}, /* Channel 6 */
        {0x1a06, 1, channels_ + 6}, /* Channel 7 */
        {0x1a07, 1, channels_ + 7}, /* Channel 8 */
    };
    ec_sync_info_t syncs_[2] = {
        {0, EC_DIR_INPUT, 8, pdos_ + 0, EC_WD_DISABLE},
        {0xff}
    };
    DomainMap domains_ = {
        {0, {0} }
    };
};
}  // namespace ethercat_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ethercat_plugins::Beckhoff_EL1008, ethercat_interface::EcSlave)
PLUGINLIB_EXPORT_CLASS(ethercat_plugins::Beckhoff_EL1018, ethercat_interface::EcSlave)
