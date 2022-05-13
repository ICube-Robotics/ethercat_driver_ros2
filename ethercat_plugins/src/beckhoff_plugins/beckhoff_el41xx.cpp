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

class Beckhoff_EL4134 : public ethercat_interface::EcSlave
{
public:
    Beckhoff_EL4134() : EcSlave(0x00000002, 0x10263052) {}
    virtual ~Beckhoff_EL4134() {}
    virtual void processData(size_t index, uint8_t* domain_address){
        auto isRequested = paramters_.find("ao."+std::to_string(index+1))!= paramters_.end();
        if(isRequested){
            double data = command_interface_ptr_->at(std::stoi(paramters_["command_interface/"+paramters_["ao."+std::to_string(index+1)]]));
            if(data>10) data = 10;
            if(data<-10) data = -10;
            state_interface_ptr_->at(std::stoi(paramters_["state_interface/"+paramters_["ao."+std::to_string(index+1)]])) = data;
            int16_t dac_data = (int16_t)(data*(double)65534/20);
            EC_WRITE_S16(domain_address, dac_data);
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
private:
    ec_pdo_entry_info_t channels_[4] = {
        {0x7000, 0x01, 16}, /* Analog output */
        {0x7010, 0x01, 16}, /* Analog output */
        {0x7020, 0x01, 16}, /* Analog output */
        {0x7030, 0x01, 16}, /* Analog output */
    };
    ec_pdo_info_t pdos_[4] = {
        {0x1600, 1, channels_ + 0}, /* AO RxPDO-Map OutputsCh.1 */
        {0x1601, 1, channels_ + 1}, /* AO RxPDO-Map OutputsCh.2 */
        {0x1602, 1, channels_ + 2}, /* AO RxPDO-Map OutputsCh.3 */
        {0x1603, 1, channels_ + 3}, /* AO RxPDO-Map OutputsCh.4 */
    };
    ec_sync_info_t syncs_[2] = {
        {2, EC_DIR_OUTPUT, 4, pdos_ + 0, EC_WD_ENABLE},
        {0xff}
    };
    DomainMap domains_ = {
        {0, {0,1,2,3} }
    };
};
//---------------------------------------------------------------------------------------------------------
class Beckhoff_EL4132 : public ethercat_interface::EcSlave
{
public :
    Beckhoff_EL4132() : EcSlave(0x00000002, 0x10243052) {}
    virtual ~Beckhoff_EL4132() {}
    virtual void processData(size_t index, uint8_t* domain_address){
        auto isRequested = paramters_.find("ao."+std::to_string(index+1))!= paramters_.end();
        if(isRequested){
            double data = command_interface_ptr_->at(std::stoi(paramters_["command_interface/"+paramters_["ao."+std::to_string(index+1)]]));
            if(data>10) data = 10;
            if(data<-10) data = -10;
            state_interface_ptr_->at(std::stoi(paramters_["state_interface/"+paramters_["ao."+std::to_string(index+1)]])) = data;
            int16_t dac_data = (int16_t)(data*(double)65534/20);
            EC_WRITE_S16(domain_address, dac_data);
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
    // analog write values
    int16_t write_data_[2] = {0};
private:
    ec_pdo_entry_info_t channels_[2] = {
        {0x3001, 0x01, 16}, /* Analog Output */
        {0x3002, 0x01, 16}, /* Analog Output */
    };
    ec_pdo_info_t pdos_[2] = {
        {0x1600, 1, channels_ + 0}, /* AO RxPDO-Map OutputsCh.1 */
        {0x1601, 1, channels_ + 1}, /* AO RxPDO-Map OutputsCh.2 */
    };
    ec_sync_info_t syncs_[2] = {
        {2, EC_DIR_OUTPUT, 2, pdos_ + 0, EC_WD_ENABLE},
        {0xff}
    };
    DomainMap domains_ = {
        {0, {0,1} }
    };
};

}  // namespace ethercat_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ethercat_plugins::Beckhoff_EL4132, ethercat_interface::EcSlave)
PLUGINLIB_EXPORT_CLASS(ethercat_plugins::Beckhoff_EL4134, ethercat_interface::EcSlave)
