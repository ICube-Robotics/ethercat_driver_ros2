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

class Beckhoff_EL7031 : public ethercat_interface::EcSlave
{
public:
    Beckhoff_EL7031() : EcSlave(0x00000002, 0x1b773052) {}
    virtual ~Beckhoff_EL7031() {}
    virtual void processData(size_t index, uint8_t* domain_address){

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

        return true;
    }
    
private:

    ec_pdo_entry_info_t channels_[20] = {
        // 0x1602, stepper control (0)
        {0x7010, 0x01, 1},    // Enable
        {0x7010, 0x02, 1},    // Reset 
        {0x7010, 0x03, 1},    // Reduce torque 
        {0x0000, 0x00, 5},    //   spacer 
        {0x0000, 0x00, 8},    //   spacer 
        // 0x1603, stepper pos (5)
        {0x7010, 0x11, 32},   // Target position
        // 0x1a03, stepper status (6)
        {0x6010, 0x01, 1},    // Ready to enable 
        {0x6010, 0x02, 1},    // Ready 
        {0x6010, 0x03, 1},    // Warning 
        {0x6010, 0x04, 1},    // Error 
        {0x6010, 0x05, 1},    // Moving positive 
        {0x6010, 0x06, 1},    // Moving negative 
        {0x6010, 0x07, 1},    // Torque reduced 
        {0x0000, 0x00, 1},    //   spacer 
        {0x0000, 0x00, 3},    //   spacer
        {0x6010, 0x0c, 1},    // Digital input 1 
        {0x6010, 0x0d, 1},    // Digital input 2 
        {0x1c32, 0x20, 1},    // Sync error 
        {0x0000, 0x00, 1},    //   spacer 
        {0x1803, 0x09, 1},    // *** unknown ***
    };
    ec_pdo_info_t pdos_[5] = {
        {0x1602, 5, channels_ + 0},
        {0x1603, 1, channels_ + 5},
        {0x1a03, 14, channels_ + 6},
    };
    ec_sync_info_t syncs_[5] = {
        {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
        {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
        {2, EC_DIR_OUTPUT, 2, pdos_ + 0, EC_WD_DISABLE},
        {3, EC_DIR_INPUT, 1, pdos_ + 2, EC_WD_DISABLE},
        {0xff}

    };
    DomainMap domains_ = {
        {0, {0} }
    };
};

}  // namespace ethercat_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ethercat_plugins::Beckhoff_EL7031, ethercat_interface::EcSlave)