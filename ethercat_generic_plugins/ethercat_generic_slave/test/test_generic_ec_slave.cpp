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

#include <map>
#include <pluginlib/class_loader.hpp>
#include "ethercat_interface/ec_slave.hpp"
#include "test_generic_ec_slave.hpp"

const char test_slave_config[] =
  R"(
# Configuration file for Test Slave
vendor_id: 0x00000011
product_id: 0x07030924
assign_activate: 0x0321  # DC Synch register
sdo:  # sdo data to be transferred at slave startup
  - {index: 0x60C2, sub_index: 1, type: int8, value: 10}
  - {index: 0x60C2, sub_index: 2, type: int8, value: -3}
  - {index: 0x6098, sub_index: 0, type: int8, value: 35}
  - {index: 0x6099, sub_index: 0, type: int32, value: 0}
rpdo:  # Receive PDO Mapping
  - index: 0x1607
    channels:
      - {index: 0x607a, sub_index: 0, type: int32, command_interface: position, default: .nan}
      - {index: 0x60ff, sub_index: 0, type: int32, command_interface: velocity, default: 0}
      - {index: 0x6071, sub_index: 0, type: int16, command_interface: effort, default: -5, factor: 2, offset: 10}
      - {index: 0x6072, sub_index: 0, type: int16, command_interface: ~, default: 1000}
      - {index: 0x6040, sub_index: 0, type: uint16, command_interface: ~, default: 0}
      - {index: 0x6060, sub_index: 0, type: int8, command_interface: ~, default: 8}
tpdo:  # Transmit PDO Mapping
  - index: 0x1a07
    channels:
      - {index: 0x6064, sub_index: 0, type: int32, state_interface: position}
      - {index: 0x606c, sub_index: 0, type: int32, state_interface: velocity}
      - {index: 0x6077, sub_index: 0, type: int16, state_interface: effort, factor: 5, offset: 15}
      - {index: 0x6041, sub_index: 0, type: uint16, state_interface: ~}
      - {index: 0x6061, sub_index: 0, type: int8, state_interface: ~}
  - index: 0x1a45
    channels:
      - {index: 0x2205, sub_index: 1, type: int16, state_interface: analog_input1}
      - {index: 0x2205, sub_index: 2, type: int16, state_interface: analog_input2}
sm:  # Sync Manager
  - {index: 0, type: output, pdo: ~, watchdog: disable}
  - {index: 1, type: input, pdo: ~, watchdog: disable}
  - {index: 2, type: output, pdo: rpdo, watchdog: enable}
  - {index: 3, type: input, pdo: tpdo, watchdog: disable}
)";

void GenericEcSlaveTest::SetUp()
{
  plugin_ = std::make_unique<FriendGenericEcSlave>();
}

void GenericEcSlaveTest::TearDown()
{
  plugin_.reset(nullptr);
}

TEST_F(GenericEcSlaveTest, SlaveSetupNoSlaveConfig)
{
  SetUp();
  std::vector<double> state_interface = {0};
  std::vector<double> command_interface = {0};
  std::unordered_map<std::string, std::string> slave_paramters;
  // setup failed, 'slave_config' parameter not set
  ASSERT_EQ(
    plugin_->setup_slave(
      slave_paramters,
      &state_interface,
      &command_interface
    ),
    false
  );
}

TEST_F(GenericEcSlaveTest, SlaveSetupMissingFileSlaveConfig)
{
  SetUp();
  std::vector<double> state_interface = {0};
  std::vector<double> command_interface = {0};
  std::unordered_map<std::string, std::string> slave_paramters;
  slave_paramters["slave_config"] = "slave_config.yaml";
  // setup failed, 'slave_config.yaml' file not set
  ASSERT_EQ(
    plugin_->setup_slave(
      slave_paramters,
      &state_interface,
      &command_interface
    ),
    false
  );
}

TEST_F(GenericEcSlaveTest, SlaveSetupSlaveFromConfig)
{
  SetUp();
  ASSERT_EQ(
    plugin_->setup_from_config(YAML::Load(test_slave_config)),
    true
  );
  ASSERT_EQ(plugin_->vendor_id_, 0x00000011u);
  ASSERT_EQ(plugin_->product_id_, 0x07030924u);
  ASSERT_EQ(plugin_->assign_activate_, 0x0321u);

  ASSERT_EQ(plugin_->get_pdo_config()[0].pdo_channel_config[1].interface_name, "velocity");
  ASSERT_EQ(plugin_->get_pdo_config()[0].pdo_channel_config[2].factor, 2);
  ASSERT_EQ(plugin_->get_pdo_config()[0].pdo_channel_config[2].offset, 10);
  ASSERT_EQ(plugin_->get_pdo_config()[0].pdo_channel_config[3].default_value, 1000);
  ASSERT_TRUE(std::isnan(plugin_->get_pdo_config()[0].pdo_channel_config[0].default_value));
  ASSERT_EQ(plugin_->get_pdo_config()[0].pdo_channel_config[4].interface_name, "null");
  ASSERT_EQ(plugin_->get_pdo_config()[2].pdo_channel_config[1].interface_name, "analog_input2");
  ASSERT_EQ(plugin_->get_pdo_config()[0].pdo_channel_config[4].data_type, "uint16");
}

TEST_F(GenericEcSlaveTest, EcReadTPDOToStateInterface)
{
  SetUp();
  std::vector<double> state_interface = {0, 0};
  std::unordered_map<std::string, std::string> slave_paramters;
  slave_paramters["state_interface/effort"] = "1";

  plugin_->state_interface_ptr_ = &state_interface;
  plugin_->paramters_ = slave_paramters;
  plugin_->setup_from_config(YAML::Load(test_slave_config));
  plugin_->setup_interface_mapping();

  ASSERT_EQ(plugin_->get_pdo_config().size(), 3u);
  ASSERT_EQ(plugin_->get_pdo_config()[1].pdo_channel_config[2].interface_index, 1);
  uint8_t domain_address[2];
  write_s16(domain_address, 42);
  plugin_->process_data(1, 2, domain_address);
  ASSERT_EQ(plugin_->state_interface_ptr_->at(1), 5 * 42 + 15);
}

TEST_F(GenericEcSlaveTest, EcWriteRPDOFromCommandInterface)
{
  SetUp();
  std::vector<double> command_interface = {0, 42};
  std::unordered_map<std::string, std::string> slave_paramters;
  slave_paramters["command_interface/effort"] = "1";

  plugin_->command_interface_ptr_ = &command_interface;
  plugin_->paramters_ = slave_paramters;
  plugin_->setup_from_config(YAML::Load(test_slave_config));
  plugin_->setup_interface_mapping();

  ASSERT_EQ(plugin_->get_pdo_config()[0].pdo_channel_config[2].interface_index, 1);
  uint8_t domain_address[2];
  plugin_->process_data(0, 2, domain_address);
  ASSERT_EQ(plugin_->get_pdo_config()[0].pdo_channel_config[2].last_value, 2 * 42 + 10);
  ASSERT_EQ(read_s16(domain_address), 2 * 42 + 10);
}

TEST_F(GenericEcSlaveTest, EcWriteRPDODefaultValue)
{
  SetUp();
  plugin_->setup_from_config(YAML::Load(test_slave_config));
  plugin_->setup_interface_mapping();

  uint8_t domain_address[2];
  plugin_->process_data(0, 2, domain_address);
  ASSERT_EQ(plugin_->get_pdo_config()[0].pdo_channel_config[2].last_value, -5);
  ASSERT_EQ(read_s16(domain_address), -5);
}

TEST_F(GenericEcSlaveTest, SlaveSetupSDOConfig)
{
  SetUp();
  plugin_->setup_from_config(YAML::Load(test_slave_config));
  ASSERT_EQ(plugin_->get_sdo_config()[0].index, 0x60C2);
  ASSERT_EQ(plugin_->get_sdo_config()[0].sub_index, 1);
  ASSERT_EQ(plugin_->get_sdo_config()[1].sub_index, 2);
  ASSERT_EQ(plugin_->get_sdo_config()[0].data_size(), 1ul);
  ASSERT_EQ(plugin_->get_sdo_config()[0].data, 10);
  ASSERT_EQ(plugin_->get_sdo_config()[2].index, 0x6098);
  ASSERT_EQ(plugin_->get_sdo_config()[3].data_type, "int32");
  ASSERT_EQ(plugin_->get_sdo_config()[3].data_size(), 4ul);
}

TEST_F(GenericEcSlaveTest, SlaveSetupSyncManagerConfig)
{
  SetUp();
  plugin_->setup_from_config(YAML::Load(test_slave_config));
  ASSERT_EQ(plugin_->get_sm_config().size(), 4ul);
  ASSERT_EQ(plugin_->get_sm_config()[0].index, 0);
  ASSERT_EQ(plugin_->get_sm_config()[0].type, 0);
  ASSERT_EQ(plugin_->get_sm_config()[0].watchdog, -1);
  ASSERT_EQ(plugin_->get_sm_config()[0].pdo_name, "null");
  ASSERT_EQ(plugin_->get_sm_config()[2].pdo_name, "rpdo");
  ASSERT_EQ(plugin_->get_sm_config()[2].watchdog, 1);
}
