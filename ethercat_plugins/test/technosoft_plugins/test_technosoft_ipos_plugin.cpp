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
#include "test_technosoft_ipos_plugin.hpp"

void TechnosoftIPOSTest::SetUp()
{
  plugin_ = std::make_unique<FriendTechnosoftIPOS>();
}

void TechnosoftIPOSTest::TearDown()
{
  plugin_.reset(nullptr);
}

TEST_F(TechnosoftIPOSTest, CommandPosition)
{
  SetUp();
  std::unordered_map<std::string, std::string> slave_paramters;
  slave_paramters["command_interface/position"] = "0";
  std::vector<double> state_interface(1, 0);
  std::vector<double> command_interface(1, 100);

  plugin_->setupSlave(slave_paramters, &state_interface, &command_interface);
  uint8_t domain_address = 0;
  plugin_->mode_of_operation_display_ = 8;
  plugin_->processData(1, &domain_address);

  ASSERT_EQ(plugin_->isTargetPositionRequired, true);
  ASSERT_EQ(plugin_->mode_of_operation_display_, ModeOfOperation::MODE_CYCLIC_SYNC_POSITION);
  ASSERT_EQ(plugin_->cii_target_position, 0);
  ASSERT_EQ(plugin_->command_interface_ptr_->at(plugin_->cii_target_position), 100);
  ASSERT_EQ(plugin_->target_position_, 100);
}

TEST_F(TechnosoftIPOSTest, CommandVelocity)
{
  SetUp();
  std::unordered_map<std::string, std::string> slave_paramters;
  slave_paramters["command_interface/velocity"] = "0";
  std::vector<double> state_interface(1, 0);
  std::vector<double> command_interface(1, 100);

  plugin_->setupSlave(slave_paramters, &state_interface, &command_interface);
  uint8_t domain_address = 0;
  plugin_->mode_of_operation_display_ = 9;
  plugin_->processData(3, &domain_address);

  ASSERT_EQ(plugin_->isTargetVelocityRequired, true);
  ASSERT_EQ(plugin_->mode_of_operation_display_, ModeOfOperation::MODE_CYCLIC_SYNC_VELOCITY);
  ASSERT_EQ(plugin_->cii_target_velocity, 0);
  ASSERT_EQ(plugin_->command_interface_ptr_->at(plugin_->cii_target_velocity), 100);
  ASSERT_EQ(plugin_->target_velocity_, 100);
}

TEST_F(TechnosoftIPOSTest, ModeOfOperation)
{
  SetUp();
  std::unordered_map<std::string, std::string> slave_paramters;
  slave_paramters["mode_of_operation"] = "9";
  std::vector<double> state_interface(1, 0);
  std::vector<double> command_interface(1, 100);

  plugin_->setupSlave(slave_paramters, &state_interface, &command_interface);
  uint8_t domain_address = 0;
  plugin_->processData(2, &domain_address);

  ASSERT_EQ(plugin_->mode_of_operation_, 9);
}
