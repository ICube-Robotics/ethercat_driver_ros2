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

    ec_pdo_entry_info_t channels_[40] = {
        {0x0000, 0x00, 1}, /* Gap */
        {0x7000, 0x02, 1}, /* Enable latch extern on positive edge */
        {0x7000, 0x03, 1}, /* Set counter */
        {0x7000, 0x04, 1}, /* Enable latch extern on negative edge */
        {0x0000, 0x00, 4}, /* Gap */
        {0x0000, 0x00, 8}, /* Gap */
        {0x7000, 0x11, 16}, /* Set counter value */
        {0x7010, 0x01, 1}, /* Enable */
        {0x7010, 0x02, 1}, /* Reset */
        {0x7010, 0x03, 1}, /* Reduce torque */
        {0x0000, 0x00, 5}, /* Gap */
        {0x0000, 0x00, 8}, /* Gap */
        {0x7010, 0x21, 16}, /* Velocity */
        {0x0000, 0x00, 1}, /* Gap */
        {0x6000, 0x02, 1}, /* Latch extern valid */
        {0x6000, 0x03, 1}, /* Set counter done */
        {0x6000, 0x04, 1}, /* Counter underflow */
        {0x6000, 0x05, 1}, /* Counter overflow */
        {0x0000, 0x00, 3}, /* Gap */
        {0x0000, 0x00, 4}, /* Gap */
        {0x6000, 0x0d, 1}, /* Status of extern latch */
        {0x6000, 0x0e, 1}, /* Sync error */
        {0x0000, 0x00, 1}, /* Gap */
        {0x6000, 0x10, 1}, /* TxPDO Toggle */
        {0x6000, 0x11, 16}, /* Counter value */
        {0x6000, 0x12, 16}, /* Latch value */
        {0x6010, 0x01, 1}, /* Ready to enable */
        {0x6010, 0x02, 1}, /* Ready */
        {0x6010, 0x03, 1}, /* Warning */
        {0x6010, 0x04, 1}, /* Error */
        {0x6010, 0x05, 1}, /* Moving positive */
        {0x6010, 0x06, 1}, /* Moving negative */
        {0x6010, 0x07, 1}, /* Torque reduced */
        {0x0000, 0x00, 1}, /* Gap */
        {0x0000, 0x00, 3}, /* Gap */
        {0x6010, 0x0c, 1}, /* Digital input 1 */
        {0x6010, 0x0d, 1}, /* Digital input 2 */
        {0x6010, 0x0e, 1}, /* Sync error */
        {0x0000, 0x00, 1}, /* Gap */
        {0x6010, 0x10, 1}, /* TxPDO Toggle */
    };
    ec_pdo_info_t pdos_[5] = {
        {0x1600, 7, channels_ + 0}, /* ENC RxPDO-Map Control compact */
        {0x1602, 5, channels_ + 7}, /* STM RxPDO-Map Control */
        {0x1604, 1, channels_ + 12}, /* STM RxPDO-Map Velocity */
        {0x1a00, 13, channels_ + 13}, /* ENC TxPDO-Map Status compact */
        {0x1a03, 14, channels_ + 26}, /* STM TxPDO-Map Status */

    };
    ec_sync_info_t syncs_[5] = {
        {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
        {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
        {2, EC_DIR_OUTPUT, 3, pdos_ + 0, EC_WD_DISABLE},
        {3, EC_DIR_INPUT, 2, pdos_ + 3, EC_WD_DISABLE},
        {0xff}

    };
    DomainMap domains_ = {
        {0, {0} }
    };
};

}  // namespace ethercat_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ethercat_plugins::Beckhoff_EL7031, ethercat_interface::EcSlave)