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

#ifndef GENERIC_PLUGINS__TEST_CIA402_DRIVE_PLUGIN_HPP_
#define GENERIC_PLUGINS__TEST_CIA402_DRIVE_PLUGIN_HPP_

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include "gmock/gmock.h"

#include "ethercat_plugins/generic_plugins/cia402_drive.hpp"

// subclassing and friending so we can access member variables
class FriendCiA402Drive : public ethercat_plugins::CiA402Drive
{
  FRIEND_TEST(CiA402DriveTest, SlaveSetupDriveFromConfig);
  FRIEND_TEST(CiA402DriveTest, SlaveSetupPdoChannels);
  FRIEND_TEST(CiA402DriveTest, SlaveSetupSyncs);
  FRIEND_TEST(CiA402DriveTest, SlaveSetupDomains);
  FRIEND_TEST(CiA402DriveTest, EcReadRPDOToStateInterface);
  FRIEND_TEST(CiA402DriveTest, EcWriteTPDOFromCommandInterface);
  FRIEND_TEST(CiA402DriveTest, EcWriteTPDODefaultValue);
  FRIEND_TEST(CiA402DriveTest, FaultReset);
};

class CiA402DriveTest : public ::testing::Test
{
public:
  void SetUp();
  void TearDown();

protected:
  std::unique_ptr<FriendCiA402Drive> plugin_;
};

#endif  // GENERIC_PLUGINS__TEST_CIA402_DRIVE_PLUGIN_HPP_
