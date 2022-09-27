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

class ATI_Mini40 : public ethercat_interface::EcSlave
{
public:
    ATI_Mini40() : EcSlave(0x00000732, 0x26483052) {}
    virtual ~ATI_Mini40() {}
    virtual void processData(size_t index, uint8_t* domain_address){
        if(sii_ft_[index] >= 0){
            double data = (double)EC_READ_S32(domain_address);
            state_interface_ptr_->at(sii_ft_[index]) = data;
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

        if(paramters_.find("force.x")!= paramters_.end()){
            if(paramters_.find("state_interface/"+paramters_["force.x"]) != paramters_.end()){
                sii_ft_[0] = std::stoi(paramters_["state_interface/"+paramters_["force.x"]]);
            }
        }
        if(paramters_.find("force.y")!= paramters_.end()){
            if(paramters_.find("state_interface/"+paramters_["force.y"]) != paramters_.end()){
                sii_ft_[1] = std::stoi(paramters_["state_interface/"+paramters_["force.y"]]);
            }
        }
        if(paramters_.find("force.z")!= paramters_.end()){
            if(paramters_.find("state_interface/"+paramters_["force.z"]) != paramters_.end()){
                sii_ft_[2] = std::stoi(paramters_["state_interface/"+paramters_["force.z"]]);
            }
        }
        if(paramters_.find("torque.x")!= paramters_.end()){
            if(paramters_.find("state_interface/"+paramters_["torque.x"]) != paramters_.end()){
                sii_ft_[3] = std::stoi(paramters_["state_interface/"+paramters_["torque.x"]]);
            }
        }
        if(paramters_.find("torque.y")!= paramters_.end()){
            if(paramters_.find("state_interface/"+paramters_["torque.y"]) != paramters_.end()){
                sii_ft_[4] = std::stoi(paramters_["state_interface/"+paramters_["torque.y"]]);
            }
        }
        if(paramters_.find("torque.z")!= paramters_.end()){
            if(paramters_.find("state_interface/"+paramters_["torque.z"]) != paramters_.end()){
                sii_ft_[5] = std::stoi(paramters_["state_interface/"+paramters_["torque.z"]]);
            }
        }

        return true;
    }
private:
    int sii_ft_[6] = {-1,-1,-1,-1,-1,-1};

    ec_pdo_entry_info_t channels_[10] = {
        {0x7010, 0x01, 32}, /* Control 1 */
        {0x7010, 0x02, 32}, /* Control 2 */
        {0x6000, 0x01, 32}, /* Fx/Gage0 */
        {0x6000, 0x02, 32}, /* Fy/Gage1 */
        {0x6000, 0x03, 32}, /* Fz/Gage2 */
        {0x6000, 0x04, 32}, /* Tx/Gage3 */
        {0x6000, 0x05, 32}, /* Ty/Gage3 */
        {0x6000, 0x06, 32}, /* Tz/Gage3 */
        {0x6010, 0x00, 32}, /* SubIndex 000 */
        {0x6020, 0x00, 32}, /* SubIndex 000 */
    };
    ec_pdo_info_t pdos_[2] = {
        {0x1601, 2, channels_ + 0}, /* DO RxPDO-Map */
        {0x1a00, 8, channels_ + 2}, /* DI TxPDO-Map */

    };
    ec_sync_info_t syncs_[5] = {
      	{0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
        {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
        {2, EC_DIR_OUTPUT, 1, pdos_ + 0, EC_WD_ENABLE},
        {3, EC_DIR_INPUT, 1, pdos_ + 1, EC_WD_DISABLE},
        {0xff}

    };
    DomainMap domains_ = {
        {0, {2,3,4,5,6,7} }
    };
};

}  // namespace ethercat_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ethercat_plugins::ATI_Mini40, ethercat_interface::EcSlave)