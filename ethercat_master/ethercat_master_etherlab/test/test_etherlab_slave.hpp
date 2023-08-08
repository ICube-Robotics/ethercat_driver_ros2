// Copyright 2023 ICUBE Laboratory, University of Strasbourg
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
//
// Author: Maciej Bednarczyk (mcbed.robotics@gmail.com)

#ifndef TEST_ETHERLAB_SLAVE_HPP_
#define TEST_ETHERLAB_SLAVE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <limits>

#include "gmock/gmock.h"
#include "ethercat_interface/ec_buffer_tools.h"
#include "ethercat_interface/ec_slave.hpp"
#include "ethercat_master/ec_slave_etherlab.hpp"


// subclassing and friending so we can access member variables
class FriendEtherlabSlave : public ethercat_master::EtherlabSlave
{
public:
  explicit FriendEtherlabSlave(ethercat_interface::EcSlave * slave)
  : EtherlabSlave(slave) {}

private:
  FRIEND_TEST(EtherlabSlaveTest, SlaveSetup);
  FRIEND_TEST(EtherlabSlaveTest, SlaveSetupPdoChannels);
  FRIEND_TEST(EtherlabSlaveTest, SlaveSetupSyncs);
  FRIEND_TEST(EtherlabSlaveTest, SlaveSetupDomains);
  FRIEND_TEST(EtherlabSlaveTest, EcReadTPDOToStateInterface);
  FRIEND_TEST(EtherlabSlaveTest, EcWriteRPDOFromCommandInterface);
  FRIEND_TEST(EtherlabSlaveTest, EcWriteRPDODefaultValue);
  FRIEND_TEST(EtherlabSlaveTest, SlaveSetupSDOConfig);
  FRIEND_TEST(EtherlabSlaveTest, SlaveSetupSyncManagerConfig);
};

class EtherlabSlaveTest : public ::testing::Test
{
public:
  void SetUp();
  void TearDown();

protected:
  std::unique_ptr<FriendEtherlabSlave> etherlab_slave_;
  std::shared_ptr<ethercat_interface::EcSlave> test_slave_ptr_;
};

class TestSlave : public ethercat_interface::EcSlave
{
public:
  TestSlave()
  {
    vendor_id_ = 0x00000011;
    product_id_ = 0x07030924;
    assign_activate_ = 0x0321;

    sm_config_.push_back(ethercat_interface::SMConfig(0, 0, "null", -1));
    sm_config_.push_back(ethercat_interface::SMConfig(1, 1, "null", -1));
    sm_config_.push_back(ethercat_interface::SMConfig(2, 0, "rpdo", 1));
    sm_config_.push_back(ethercat_interface::SMConfig(3, 1, "tpdo", -1));

    sdo_config_.push_back(ethercat_interface::SdoConfigEntry(0x60C2, 1, "int8", 10));
    sdo_config_.push_back(ethercat_interface::SdoConfigEntry(0x60C2, 2, "int8", -3));
    sdo_config_.push_back(ethercat_interface::SdoConfigEntry(0x6098, 0, "int8", 35));
    sdo_config_.push_back(ethercat_interface::SdoConfigEntry(0x6099, 0, "int32", 0));

    ethercat_interface::pdo_mapping_t pdo_mapping;
    pdo_mapping.index = 0x1607;
    pdo_mapping.pdo_type = ethercat_interface::RPDO;
    pdo_mapping.pdo_channel_config.push_back(
      ethercat_interface::EcPdoChannelManager(
        0x607a, 0, ethercat_interface::RPDO, "int32",
        "position"));
    pdo_mapping.pdo_channel_config.push_back(
      ethercat_interface::EcPdoChannelManager(
        0x60ff, 0, ethercat_interface::RPDO, "int32",
        "velocity", 255, 0));
    pdo_mapping.pdo_channel_config.push_back(
      ethercat_interface::EcPdoChannelManager(
        0x6071, 0, ethercat_interface::RPDO, "int16",
        "effort", 255, -5, 2, 10));
    pdo_mapping.pdo_channel_config.push_back(
      ethercat_interface::EcPdoChannelManager(
        0x6072, 0, ethercat_interface::RPDO, "int16", "null",
        255, 1000));
    pdo_mapping.pdo_channel_config.push_back(
      ethercat_interface::EcPdoChannelManager(
        0x6040, 0, ethercat_interface::RPDO, "uint16", "null",
        255, 0));
    pdo_mapping.pdo_channel_config.push_back(
      ethercat_interface::EcPdoChannelManager(
        0x6060, 0, ethercat_interface::RPDO, "int8", "null",
        255, 8));
    pdo_config_.push_back(pdo_mapping);

    pdo_mapping.pdo_channel_config.clear();
    pdo_mapping.index = 0x1a07;
    pdo_mapping.pdo_type = ethercat_interface::TPDO;
    pdo_mapping.pdo_channel_config.push_back(
      ethercat_interface::EcPdoChannelManager(
        0x6064, 0, ethercat_interface::TPDO, "int32",
        "position"));
    pdo_mapping.pdo_channel_config.push_back(
      ethercat_interface::EcPdoChannelManager(
        0x606c, 0, ethercat_interface::TPDO, "int32",
        "velocity"));
    pdo_mapping.pdo_channel_config.push_back(
      ethercat_interface::EcPdoChannelManager(
        0x6077, 0, ethercat_interface::TPDO, "int16", "effort",
        255, std::numeric_limits<double>::quiet_NaN(), 5, 15)
    );
    pdo_mapping.pdo_channel_config.push_back(
      ethercat_interface::EcPdoChannelManager(
        0x6041, 0, ethercat_interface::TPDO, "uint16",
        "null"));
    pdo_mapping.pdo_channel_config.push_back(
      ethercat_interface::EcPdoChannelManager(0x6061, 0, ethercat_interface::TPDO, "int8", "null"));
    pdo_config_.push_back(pdo_mapping);

    pdo_mapping.pdo_channel_config.clear();
    pdo_mapping.index = 0x1a45;
    pdo_mapping.pdo_type = ethercat_interface::TPDO;
    pdo_mapping.pdo_channel_config.push_back(
      ethercat_interface::EcPdoChannelManager(
        0x2205, 0x01, ethercat_interface::TPDO, "int16",
        "analog_input1"));
    pdo_mapping.pdo_channel_config.push_back(
      ethercat_interface::EcPdoChannelManager(
        0x2205, 0x02, ethercat_interface::TPDO, "int16",
        "analog_input2"));
    pdo_config_.push_back(pdo_mapping);
  }
  ~TestSlave() {}
};

#endif  // TEST_ETHERLAB_SLAVE_HPP_
