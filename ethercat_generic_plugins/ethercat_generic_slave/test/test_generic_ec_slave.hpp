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

#ifndef TEST_GENERIC_EC_SLAVE_HPP_
#define TEST_GENERIC_EC_SLAVE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include "gmock/gmock.h"
#include "ethercat_interface/ec_buffer_tools.h"

#include "ethercat_generic_plugins/generic_ec_slave.hpp"

// subclassing and friending so we can access member variables
class FriendGenericEcSlave : public ethercat_generic_plugins::GenericEcSlave
{
  FRIEND_TEST(GenericEcSlaveTest, SlaveSetupSlaveFromConfig);
  // FRIEND_TEST(GenericEcSlaveTest, SlaveSetupPdoChannels);
  // FRIEND_TEST(GenericEcSlaveTest, SlaveSetupSyncs);
  // FRIEND_TEST(GenericEcSlaveTest, SlaveSetupDomains);
  FRIEND_TEST(GenericEcSlaveTest, EcReadTPDOToStateInterface);
  FRIEND_TEST(GenericEcSlaveTest, EcWriteRPDOFromCommandInterface);
  FRIEND_TEST(GenericEcSlaveTest, EcWriteRPDODefaultValue);
  FRIEND_TEST(GenericEcSlaveTest, SlaveSetupSDOConfig);
  FRIEND_TEST(GenericEcSlaveTest, SlaveSetupSyncManagerConfig);
};

class GenericEcSlaveTest : public ::testing::Test
{
public:
  void SetUp();
  void TearDown();

protected:
  std::unique_ptr<FriendGenericEcSlave> plugin_;
};

#endif  // TEST_GENERIC_EC_SLAVE_HPP_
