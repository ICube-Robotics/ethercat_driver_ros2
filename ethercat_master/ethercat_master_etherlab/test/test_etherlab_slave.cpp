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
#include "test_etherlab_slave.hpp"

void EtherlabSlaveTest::SetUp()
{
  auto test_slave_ptr = std::make_shared<TestSlave>();
  etherlab_slave_ = std::make_unique<FriendEtherlabSlave>(test_slave_ptr);
}

void EtherlabSlaveTest::TearDown()
{
}

TEST_F(EtherlabSlaveTest, SlaveSetup)
{
  SetUp();
  std::vector<double> state_interface = {0};
  std::vector<double> command_interface = {0};
  std::unordered_map<std::string, std::string> slave_paramters;

  ASSERT_EQ(etherlab_slave_->get_vendor_id(), 0x00000011u);
  ASSERT_EQ(etherlab_slave_->get_product_id(), 0x07030924u);
  ASSERT_EQ(etherlab_slave_->dc_sync(), 0x0321);

  ASSERT_EQ(etherlab_slave_->get_sm_config().size(), 4u);
  ASSERT_EQ(etherlab_slave_->get_sdo_config().size(), 4u);
  ASSERT_EQ(etherlab_slave_->get_pdo_config().size(), 3u);

  ASSERT_EQ(etherlab_slave_->get_pdo_config()[0].pdo_channel_config[1].interface_name, "velocity");
  ASSERT_EQ(etherlab_slave_->get_pdo_config()[0].pdo_channel_config[2].factor, 2);
  ASSERT_EQ(etherlab_slave_->get_pdo_config()[0].pdo_channel_config[2].offset, 10);
  ASSERT_EQ(etherlab_slave_->get_pdo_config()[0].pdo_channel_config[3].default_value, 1000);
  ASSERT_TRUE(std::isnan(etherlab_slave_->get_pdo_config()[0].pdo_channel_config[0].default_value));
  ASSERT_EQ(etherlab_slave_->get_pdo_config()[0].pdo_channel_config[4].interface_name, "null");
  ASSERT_EQ(
    etherlab_slave_->get_pdo_config()[2].pdo_channel_config[1].interface_name, "analog_input2");
  ASSERT_EQ(etherlab_slave_->get_pdo_config()[0].pdo_channel_config[4].data_type, "uint16");

  ASSERT_EQ(
    etherlab_slave_->setup_slave(
      slave_paramters,
      &state_interface,
      &command_interface
    ),
    true
  );

  ASSERT_EQ(etherlab_slave_->rpdos_.size(), 1u);
  ASSERT_EQ(etherlab_slave_->rpdos_[0].index, 0x1607u);

  ASSERT_EQ(etherlab_slave_->tpdos_.size(), 2u);
  ASSERT_EQ(etherlab_slave_->tpdos_[0].index, 0x1a07u);
  ASSERT_EQ(etherlab_slave_->tpdos_[1].index, 0x1a45u);
}

TEST_F(EtherlabSlaveTest, SlaveSetupPdoChannels)
{
  SetUp();
  std::vector<double> state_interface = {0};
  std::vector<double> command_interface = {0};
  std::unordered_map<std::string, std::string> slave_paramters;
  etherlab_slave_->setup_slave(
    slave_paramters,
    &state_interface,
    &command_interface
  );

  std::vector<ec_pdo_entry_info_t> channels(
    etherlab_slave_->channels(),
    etherlab_slave_->channels() + etherlab_slave_->all_channels_.size()
  );

  ASSERT_EQ(channels.size(), 13u);
  ASSERT_EQ(channels[0].index, 0x607au);
  ASSERT_EQ(channels[11].index, 0x2205u);
  ASSERT_EQ(channels[11].subindex, 0x01u);
}

TEST_F(EtherlabSlaveTest, SlaveSetupSyncs)
{
  SetUp();
  std::vector<double> state_interface = {0};
  std::vector<double> command_interface = {0};
  std::unordered_map<std::string, std::string> slave_paramters;
  etherlab_slave_->setup_slave(
    slave_paramters,
    &state_interface,
    &command_interface
  );

  std::vector<ec_sync_info_t> syncs(
    etherlab_slave_->syncs(),
    etherlab_slave_->syncs() + etherlab_slave_->sync_size()
  );

  ASSERT_EQ(syncs.size(), 5u);
  ASSERT_EQ(syncs[1].index, 1u);
  ASSERT_EQ(syncs[1].dir, EC_DIR_INPUT);
  ASSERT_EQ(syncs[1].n_pdos, 0u);
  ASSERT_EQ(syncs[1].watchdog_mode, EC_WD_DISABLE);
  ASSERT_EQ(syncs[2].dir, EC_DIR_OUTPUT);
  ASSERT_EQ(syncs[2].n_pdos, 1u);
  ASSERT_EQ(syncs[3].index, 3u);
  ASSERT_EQ(syncs[3].dir, EC_DIR_INPUT);
  ASSERT_EQ(syncs[3].n_pdos, 2u);
  ASSERT_EQ(syncs[3].watchdog_mode, EC_WD_DISABLE);
}

TEST_F(EtherlabSlaveTest, SlaveSetupDomains)
{
  SetUp();
  std::vector<double> state_interface = {0};
  std::vector<double> command_interface = {0};
  std::unordered_map<std::string, std::string> slave_paramters;
  etherlab_slave_->setup_slave(
    slave_paramters,
    &state_interface,
    &command_interface
  );

  std::map<unsigned int, std::vector<unsigned int>> domains;
  etherlab_slave_->domains(domains);

  ASSERT_EQ(domains[0].size(), 13u);
  ASSERT_EQ(domains[0][0], 0u);
  ASSERT_EQ(domains[0][12], 12u);
}

TEST_F(EtherlabSlaveTest, EcReadTPDOToStateInterface)
{
  SetUp();
  std::vector<double> state_interface = {0, 0};
  std::vector<double> command_interface = {0};
  std::unordered_map<std::string, std::string> slave_paramters;
  slave_paramters["state_interface/effort"] = "1";

  etherlab_slave_->setup_slave(
    slave_paramters,
    &state_interface,
    &command_interface
  );

  ASSERT_EQ(etherlab_slave_->get_pdo_config()[1].pdo_channel_config[2].interface_index, 1);
  uint8_t domain_address[2];
  write_s16(domain_address, 42);
  etherlab_slave_->process_data(1, 2, domain_address);
  ASSERT_EQ(etherlab_slave_->state_interface_ptr_->at(1), 5 * 42 + 15);
}

TEST_F(EtherlabSlaveTest, EcWriteRPDOFromCommandInterface)
{
  SetUp();
  std::vector<double> state_interface = {0, 0};
  std::vector<double> command_interface = {0, 42};
  std::unordered_map<std::string, std::string> slave_paramters;
  slave_paramters["command_interface/effort"] = "1";

  etherlab_slave_->setup_slave(
    slave_paramters,
    &state_interface,
    &command_interface
  );

  ASSERT_EQ(etherlab_slave_->get_pdo_config()[0].pdo_channel_config[2].interface_index, 1);
  uint8_t domain_address[2];
  etherlab_slave_->process_data(0, 2, domain_address);
  ASSERT_EQ(etherlab_slave_->get_pdo_config()[0].pdo_channel_config[2].last_value, 2 * 42 + 10);
  ASSERT_EQ(read_s16(domain_address), 2 * 42 + 10);
}

TEST_F(EtherlabSlaveTest, EcWriteRPDODefaultValue)
{
  SetUp();
  std::vector<double> state_interface = {0};
  std::vector<double> command_interface = {0};
  std::unordered_map<std::string, std::string> slave_paramters;

  etherlab_slave_->setup_slave(
    slave_paramters,
    &state_interface,
    &command_interface
  );

  uint8_t domain_address[2];
  etherlab_slave_->process_data(0, 2, domain_address);
  ASSERT_EQ(etherlab_slave_->get_pdo_config()[0].pdo_channel_config[2].last_value, -5);
  ASSERT_EQ(read_s16(domain_address), -5);
}

TEST_F(EtherlabSlaveTest, SlaveSetupSDOConfig)
{
  SetUp();
  ASSERT_EQ(etherlab_slave_->get_sdo_config()[0].index, 0x60C2);
  ASSERT_EQ(etherlab_slave_->get_sdo_config()[0].sub_index, 1);
  ASSERT_EQ(etherlab_slave_->get_sdo_config()[1].sub_index, 2);
  ASSERT_EQ(etherlab_slave_->get_sdo_config()[0].data_size(), 1ul);
  ASSERT_EQ(etherlab_slave_->get_sdo_config()[0].data, 10);
  ASSERT_EQ(etherlab_slave_->get_sdo_config()[2].index, 0x6098);
  ASSERT_EQ(etherlab_slave_->get_sdo_config()[3].data_type, "int32");
  ASSERT_EQ(etherlab_slave_->get_sdo_config()[3].data_size(), 4ul);
}

TEST_F(EtherlabSlaveTest, SlaveSetupSyncManagerConfig)
{
  SetUp();
  ASSERT_EQ(etherlab_slave_->get_sm_config().size(), 4ul);
  ASSERT_EQ(etherlab_slave_->get_sm_config()[0].index, 0);
  ASSERT_EQ(etherlab_slave_->get_sm_config()[0].type, 0);
  ASSERT_EQ(etherlab_slave_->get_sm_config()[0].watchdog, -1);
  ASSERT_EQ(etherlab_slave_->get_sm_config()[0].pdo_name, "null");
  ASSERT_EQ(etherlab_slave_->get_sm_config()[2].pdo_name, "rpdo");
  ASSERT_EQ(etherlab_slave_->get_sm_config()[2].watchdog, 1);
}
