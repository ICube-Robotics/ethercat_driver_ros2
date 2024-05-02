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
#include <limits>
#include <pluginlib/class_loader.hpp>
#include "ethercat_interface/ec_slave.hpp"
#include "test_generic_ec_cia402_drive.hpp"

const char test_drive_config[] =
  R"(
# Configuration file for Test drive
vendor_id: 0x00000011
product_id: 0x07030924
assign_activate: 0x0321  # DC Synch register
period: 100  # Hz
auto_fault_reset: false  # true = automatic fault reset, false = fault reset on rising edge command interface "reset_fault"
sdo:  # sdo data to be transferred at drive startup
  - {index: 0x60C2, sub_index: 1, type: int8, value: 10} # Set interpolation time for cyclic modes to 10 ms
  - {index: 0x60C2, sub_index: 2, type: int8, value: -3} # Set base 10-3s
rpdo:  # RxPDO
  - index: 0x1607
    channels:
      - {index: 0x607a, sub_index: 0, type: int32, command_interface: position, default: .nan}  # Target position
      - {index: 0x60ff, sub_index: 0, type: int32, command_interface: velocity, default: 0}  # Target velocity
      - {index: 0x6071, sub_index: 0, type: int16, command_interface: effort, default: -5}  # Target torque
      - {index: 0x6072, sub_index: 0, type: int16, command_interface: ~, default: 1000}  # Max torque
      - {index: 0x6040, sub_index: 0, type: uint16, command_interface: ~, default: 0}  # Control word
      - {index: 0x6060, sub_index: 0, type: int8, command_interface: mode_of_operation, default: 8}  # Mode of operation
tpdo:  # TxPDO
  - index: 0x1a07
    channels:
      - {index: 0x6064, sub_index: 0, type: int32, state_interface: position}  # Position actual value
      - {index: 0x606c, sub_index: 0, type: int32, state_interface: velocity}  # Velocity actual value
      - {index: 0x6077, sub_index: 0, type: int16, state_interface: effort}  # Torque actual value
      - {index: 0x6041, sub_index: 0, type: uint16, state_interface: ~}  # Status word
      - {index: 0x6061, sub_index: 0, type: int8, state_interface: mode_of_operation}  # Mode of operation display
  - index: 0x1a45
    channels:
      - {index: 0x2205, sub_index: 1, type: int16, state_interface: analog_input1}  # Analog input
      - {index: 0x2205, sub_index: 2, type: int16, state_interface: analog_input2}  # Analog input
)";

void EcCiA402DriveTest::SetUp()
{
  plugin_ = std::make_unique<FriendEcCiA402Drive>();
}

void EcCiA402DriveTest::TearDown()
{
  plugin_.reset(nullptr);
}

TEST_F(EcCiA402DriveTest, SlaveSetupNoDriveConfig)
{
  SetUp();
  std::vector<double> state_interface = {0};
  std::vector<double> command_interface = {0};
  std::unordered_map<std::string, std::string> slave_paramters;
  // setup failed, 'drive_config' parameter not set
  ASSERT_EQ(
    plugin_->setup_slave(
      slave_paramters,
      &state_interface,
      &command_interface
    ),
    false
  );
}

TEST_F(EcCiA402DriveTest, SlaveSetupMissingFileDriveConfig)
{
  SetUp();
  std::vector<double> state_interface = {0};
  std::vector<double> command_interface = {0};
  std::unordered_map<std::string, std::string> slave_paramters;
  slave_paramters["drive_config"] = "drive_config.yaml";
  // setup failed, 'drive_config.yaml' file not set
  ASSERT_EQ(
    plugin_->setup_slave(
      slave_paramters,
      &state_interface,
      &command_interface
    ),
    false
  );
}

TEST_F(EcCiA402DriveTest, SlaveSetupDriveFromConfig)
{
  SetUp();
  ASSERT_EQ(
    plugin_->setup_from_config(YAML::Load(test_drive_config)),
    true
  );
  ASSERT_EQ(plugin_->vendor_id_, 0x00000011u);
  ASSERT_EQ(plugin_->product_id_, 0x07030924u);
  ASSERT_EQ(plugin_->assign_activate_, 0x0321u);
  ASSERT_EQ(plugin_->auto_fault_reset_, false);

  ASSERT_EQ(plugin_->get_pdo_config()[0].pdo_channel_config[1].interface_name, "velocity");
  ASSERT_EQ(plugin_->get_pdo_config()[0].pdo_channel_config[3].default_value, 1000);
  ASSERT_TRUE(std::isnan(plugin_->get_pdo_config()[0].pdo_channel_config[0].default_value));
  ASSERT_EQ(plugin_->get_pdo_config()[0].pdo_channel_config[4].interface_name, "null");
  ASSERT_EQ(plugin_->get_pdo_config()[2].pdo_channel_config[1].interface_name, "analog_input2");
  ASSERT_EQ(plugin_->get_pdo_config()[0].pdo_channel_config[4].data_type, "uint16");
}

TEST_F(EcCiA402DriveTest, EcReadTPDOToStateInterface)
{
  SetUp();
  std::vector<double> state_interface = {0, 0};
  std::unordered_map<std::string, std::string> slave_paramters;
  slave_paramters["state_interface/effort"] = "1";

  plugin_->state_interface_ptr_ = &state_interface;
  plugin_->paramters_ = slave_paramters;
  plugin_->setup_from_config(YAML::Load(test_drive_config));
  plugin_->setup_interface_mapping();

  ASSERT_EQ(plugin_->get_pdo_config().size(), 3u);
  ASSERT_EQ(plugin_->get_pdo_config()[1].pdo_channel_config[2].interface_index, 1);
  uint8_t domain_address[2];
  write_s16(domain_address, 42);
  plugin_->process_data(1, 2, domain_address);
  ASSERT_EQ(plugin_->state_interface_ptr_->at(1), 42);
}

TEST_F(EcCiA402DriveTest, EcWriteRPDOFromCommandInterface)
{
  SetUp();
  std::vector<double> command_interface = {0, 42};
  std::unordered_map<std::string, std::string> slave_paramters;
  slave_paramters["command_interface/effort"] = "1";

  plugin_->command_interface_ptr_ = &command_interface;
  plugin_->paramters_ = slave_paramters;
  plugin_->setup_from_config(YAML::Load(test_drive_config));
  plugin_->setup_interface_mapping();

  ASSERT_EQ(plugin_->get_pdo_config()[0].pdo_channel_config[2].interface_index, 1);
  plugin_->mode_of_operation_display_ = 10;
  uint8_t domain_address[2];
  plugin_->process_data(0, 2, domain_address);
  ASSERT_EQ(plugin_->get_pdo_config()[0].pdo_channel_config[2].last_value, 42);
  ASSERT_EQ(read_s16(domain_address), 42);
}

TEST_F(EcCiA402DriveTest, EcWriteRPDODefaultValue)
{
  SetUp();
  plugin_->setup_from_config(YAML::Load(test_drive_config));
  plugin_->setup_interface_mapping();

  plugin_->mode_of_operation_display_ = 10;
  uint8_t domain_address[2];
  plugin_->process_data(0, 2, domain_address);
  ASSERT_EQ(plugin_->get_pdo_config()[0].pdo_channel_config[2].last_value, -5);
  ASSERT_EQ(read_s16(domain_address), -5);
}

// TEST_F(EcCiA402DriveTest, FaultReset)
// {
//   std::unordered_map<std::string, std::string> slave_paramters;
//   std::vector<double> command_interface = {0, 1};
//   plugin_->command_interface_ptr_ = &command_interface;
//   plugin_->setup_from_config(YAML::Load(test_drive_config));
//   plugin_->setup_interface_mapping();
//   plugin_->fault_reset_command_interface_index_ = 1;
//   plugin_->state_ = STATE_FAULT;
//   plugin_->is_operational_ = true;
//   uint8_t domain_address = 0;
//   plugin_->pdo_channels_info_[4].data_type = "";
//   ASSERT_FALSE(plugin_->last_fault_reset_command_);
//   ASSERT_FALSE(plugin_->fault_reset_);
//   ASSERT_EQ(plugin_->command_interface_ptr_->at(
//     plugin_->fault_reset_command_interface_index_), 1);
//   plugin_->process_data(4, &domain_address);
//   ASSERT_EQ(plugin_->pdo_channels_info_[4].default_value, 0b10000000);
//   plugin_->pdo_channels_info_[4].last_value = 0;
//   plugin_->process_data(4, &domain_address);
//   ASSERT_EQ(plugin_->pdo_channels_info_[4].default_value, 0b00000000);
//   command_interface[1] = 0;
//   plugin_->process_data(4, &domain_address);
//   ASSERT_EQ(plugin_->pdo_channels_info_[4].default_value, 0b00000000);
//   command_interface[1] = 2;  plugin_->process_data(4, &domain_address);
//   ASSERT_EQ(plugin_->pdo_channels_info_[4].default_value, 0b10000000);
// }

TEST_F(EcCiA402DriveTest, SwitchModeOfOperation)
{
  std::unordered_map<std::string, std::string> slave_paramters;
  std::vector<double> command_interface = {
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};
  slave_paramters["command_interface/mode_of_operation"] = "1";

  plugin_->paramters_ = slave_paramters;
  plugin_->command_interface_ptr_ = &command_interface;
  plugin_->setup_from_config(YAML::Load(test_drive_config));
  plugin_->setup_interface_mapping();

  plugin_->is_operational_ = true;
  uint8_t domain_address[2];
  plugin_->process_data(0, 5, domain_address);
  ASSERT_EQ(read_s8(domain_address), 8);
  command_interface[1] = 9;
  plugin_->process_data(0, 5, domain_address);
  plugin_->process_data(1, 4, domain_address);
  ASSERT_EQ(read_s8(domain_address), 9);
  ASSERT_EQ(plugin_->mode_of_operation_display_, 9);
}

TEST_F(EcCiA402DriveTest, EcWriteDefaultTargetPosition)
{
  std::unordered_map<std::string, std::string> slave_paramters;
  std::vector<double> command_interface = {
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};
  slave_paramters["command_interface/mode_of_operation"] = "1";

  plugin_->paramters_ = slave_paramters;
  plugin_->command_interface_ptr_ = &command_interface;
  plugin_->setup_from_config(YAML::Load(test_drive_config));
  plugin_->setup_interface_mapping();

  plugin_->is_operational_ = true;
  plugin_->mode_of_operation_display_ = 8;
  uint8_t domain_address[4];
  uint8_t domain_address_moo[2];
  plugin_->process_data(0, 5, domain_address_moo);
  plugin_->process_data(1, 4, domain_address_moo);
  ASSERT_EQ(plugin_->mode_of_operation_display_, 8);

  write_s32(domain_address, 123456);
  plugin_->process_data(1, 0, domain_address);
  ASSERT_EQ(plugin_->last_position_, 123456);

  write_s32(domain_address, 0);
  plugin_->process_data(0, 0, domain_address);
  ASSERT_EQ(read_s32(domain_address), 123456);

  command_interface[1] = 9;
  plugin_->process_data(0, 5, domain_address_moo);
  plugin_->process_data(1, 4, domain_address_moo);
  ASSERT_EQ(plugin_->mode_of_operation_display_, 9);

  write_s32(domain_address, 0);
  plugin_->process_data(0, 0, domain_address);
  ASSERT_EQ(read_s32(domain_address), 123456);

  write_s32(domain_address, 654321);
  plugin_->process_data(1, 0, domain_address);
  ASSERT_EQ(plugin_->last_position_, 654321);

  write_s32(domain_address, 0);
  plugin_->process_data(0, 0, domain_address);
  ASSERT_EQ(read_s32(domain_address), 654321);
}
