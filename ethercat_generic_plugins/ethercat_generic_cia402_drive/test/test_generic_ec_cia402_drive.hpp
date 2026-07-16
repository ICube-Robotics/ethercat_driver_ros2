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

#ifndef TEST_GENERIC_EC_CIA402_DRIVE_HPP_
#define TEST_GENERIC_EC_CIA402_DRIVE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include "gmock/gmock.h"

#include "ethercat_generic_plugins/generic_ec_cia402_drive.hpp"

// subclassing and friending so we can access member variables
class FriendEcCiA402Drive : public ethercat_generic_plugins::EcCiA402Drive
{
  FRIEND_TEST(EcCiA402DriveTest, SlaveSetupDriveFromConfig);
  FRIEND_TEST(EcCiA402DriveTest, SlaveSetupPdoChannels);
  FRIEND_TEST(EcCiA402DriveTest, SlaveSetupSyncs);
  FRIEND_TEST(EcCiA402DriveTest, SlaveSetupDomains);
  FRIEND_TEST(EcCiA402DriveTest, EcReadTPDOToStateInterface);
  FRIEND_TEST(EcCiA402DriveTest, EcWriteRPDOFromCommandInterface);
  FRIEND_TEST(EcCiA402DriveTest, EcWriteRPDODefaultValue);
  // FRIEND_TEST(EcCiA402DriveTest, FaultReset);
  FRIEND_TEST(EcCiA402DriveTest, SwitchModeOfOperation);
  FRIEND_TEST(EcCiA402DriveTest, EcWriteDefaultTargetPosition);
};

class EcCiA402DriveTest : public ::testing::Test
{
public:
  void SetUp();
  void TearDown();

protected:
  std::unique_ptr<FriendEcCiA402Drive> plugin_;
};

#endif  // TEST_GENERIC_EC_CIA402_DRIVE_HPP_
