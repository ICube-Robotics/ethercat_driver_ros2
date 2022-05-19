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

class Beckhoff_EL3102 : public ethercat_interface::EcSlave
{
public:
    Beckhoff_EL3102() : EcSlave(0x00000002, 0x0c1e3052) {}
    virtual ~Beckhoff_EL3102() {}
    virtual void processData(size_t index, uint8_t* domain_address){
        if(sii_ai_[index] >= 0){
            double data = (double)EC_READ_S16(domain_address)/32767*10;
            state_interface_ptr_->at(sii_ai_[index]) = data;
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
    virtual bool setupSlave(
                std::unordered_map<std::string, std::string> slave_paramters,
                std::vector<double> * state_interface,
                std::vector<double> * command_interface){

        state_interface_ptr_ = state_interface;
        command_interface_ptr_ = command_interface;
        paramters_ = slave_paramters;

        for(auto index = 0ul; index < 2; index++){
            if(paramters_.find("ai."+std::to_string(index+1))!= paramters_.end()){
                if(paramters_.find("state_interface/"+paramters_["ai."+std::to_string(index+1)]) != paramters_.end())
                    sii_ai_[index] = std::stoi(paramters_["state_interface/"+paramters_["ai."+std::to_string(index+1)]]);
            }
        }
        return true;
    }
private:
    int sii_ai_[2] = {-1, -1};

    ec_pdo_entry_info_t channels_[4] = {
    {0x3101, 0x01, 8}, /* Status */
    {0x3101, 0x02, 16}, /* Value */
    {0x3102, 0x01, 8}, /* Status */
    {0x3102, 0x02, 16}, /* Value */

    };
    ec_pdo_info_t pdos_[2] = {
        {0x1a00, 2, channels_ + 0}, /* TxPDO-Map Channel 1 */
        {0x1a01, 2, channels_ + 2}, /* TxPDO-Map Channel 2 */

    };
    ec_sync_info_t syncs_[5] = {
      	{0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
        {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
        {2, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
        {3, EC_DIR_INPUT, 2, pdos_ + 0, EC_WD_DISABLE},
        {0xff}
    };
    DomainMap domains_ = {
        {0, {1,3} }
    };
};
// --------------------------------------------------------------------------------------
class Beckhoff_EL3104 : public ethercat_interface::EcSlave
{
public:
    Beckhoff_EL3104() : EcSlave(0x00000002, 0x0c203052) {}
    virtual ~Beckhoff_EL3104() {}
    virtual void processData(size_t index, uint8_t* domain_address){
        if(sii_ai_[index] >= 0){
            double data = (double)EC_READ_S16(domain_address)/32767*10;
            state_interface_ptr_->at(sii_ai_[index]) = data;
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
    virtual bool setupSlave(
                std::unordered_map<std::string, std::string> slave_paramters,
                std::vector<double> * state_interface,
                std::vector<double> * command_interface){

        state_interface_ptr_ = state_interface;
        command_interface_ptr_ = command_interface;
        paramters_ = slave_paramters;

        for(auto index = 0ul; index < 4; index++){
            if(paramters_.find("ai."+std::to_string(index+1))!= paramters_.end()){
                if(paramters_.find("state_interface/"+paramters_["ai."+std::to_string(index+1)]) != paramters_.end())
                    sii_ai_[index] = std::stoi(paramters_["state_interface/"+paramters_["ai."+std::to_string(index+1)]]);
            }
        }
        return true;
    }
private:
    int sii_ai_[4] = {-1, -1, -1, -1};

    ec_pdo_entry_info_t channels_[44] = {
    {0x6000, 0x01, 1}, /* Underrange */
    {0x6000, 0x02, 1}, /* Overrange */
    {0x6000, 0x03, 2}, /* Limit 1 */
    {0x6000, 0x05, 2}, /* Limit 2 */
    {0x6000, 0x07, 1}, /* Error */
    {0x0000, 0x00, 1}, /* Gap */
    {0x0000, 0x00, 5}, /* Gap */
    {0x6000, 0x0e, 1}, /* Sync error */
    {0x6000, 0x0f, 1}, /* TxPDO State */
    {0x6000, 0x10, 1}, /* TxPDO Toggle */
    {0x6000, 0x11, 16}, /* Value */
    {0x6010, 0x01, 1}, /* Underrange */
    {0x6010, 0x02, 1}, /* Overrange */
    {0x6010, 0x03, 2}, /* Limit 1 */
    {0x6010, 0x05, 2}, /* Limit 2 */
    {0x6010, 0x07, 1}, /* Error */
    {0x0000, 0x00, 1}, /* Gap */
    {0x0000, 0x00, 5}, /* Gap */
    {0x6010, 0x0e, 1}, /* Sync error */
    {0x6010, 0x0f, 1}, /* TxPDO State */
    {0x6010, 0x10, 1}, /* TxPDO Toggle */
    {0x6010, 0x11, 16}, /* Value */
    {0x6020, 0x01, 1}, /* Underrange */
    {0x6020, 0x02, 1}, /* Overrange */
    {0x6020, 0x03, 2}, /* Limit 1 */
    {0x6020, 0x05, 2}, /* Limit 2 */
    {0x6020, 0x07, 1}, /* Error */
    {0x0000, 0x00, 1}, /* Gap */
    {0x0000, 0x00, 5}, /* Gap */
    {0x6020, 0x0e, 1}, /* Sync error */
    {0x6020, 0x0f, 1}, /* TxPDO State */
    {0x6020, 0x10, 1}, /* TxPDO Toggle */
    {0x6020, 0x11, 16}, /* Value */
    {0x6030, 0x01, 1}, /* Underrange */
    {0x6030, 0x02, 1}, /* Overrange */
    {0x6030, 0x03, 2}, /* Limit 1 */
    {0x6030, 0x05, 2}, /* Limit 2 */
    {0x6030, 0x07, 1}, /* Error */
    {0x0000, 0x00, 1}, /* Gap */
    {0x0000, 0x00, 5}, /* Gap */
    {0x6030, 0x0e, 1}, /* Sync error */
    {0x6030, 0x0f, 1}, /* TxPDO State */
    {0x6030, 0x10, 1}, /* TxPDO Toggle */
    {0x6030, 0x11, 16}, /* Value */
    };
    ec_pdo_info_t pdos_[4] = {
    {0x1a00, 11, channels_ + 0}, /* AI TxPDO-Map Standard Ch.1 */
    {0x1a02, 11, channels_ + 11}, /* AI TxPDO-Map Standard Ch.2 */
    {0x1a04, 11, channels_ + 22}, /* AI TxPDO-Map Standard Ch.3 */
    {0x1a06, 11, channels_ + 33}, /* AI TxPDO-Map Standard Ch.4 */
    };
    ec_sync_info_t syncs_[2] = {
      	{3, EC_DIR_INPUT, 4, pdos_ + 0, EC_WD_ENABLE},
        {0xff}
    };
    DomainMap domains_ = {
        {0, {10,21,32,43} }
    };
};

}  // namespace ethercat_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ethercat_plugins::Beckhoff_EL3102, ethercat_interface::EcSlave)
PLUGINLIB_EXPORT_CLASS(ethercat_plugins::Beckhoff_EL3104, ethercat_interface::EcSlave)
