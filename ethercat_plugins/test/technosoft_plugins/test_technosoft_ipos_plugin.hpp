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

#ifndef TECHNOSOFT_PLUGINS__TEST_TECHNOSOFT_IPOS_PLUGIN_HPP_
#define TECHNOSOFT_PLUGINS__TEST_TECHNOSOFT_IPOS_PLUGIN_HPP_

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include "gmock/gmock.h"

#include "ethercat_plugins/technosoft_plugins/technosoft_ipos.hpp"

// subclassing and friending so we can access member variables
class FriendTechnosoftIPOS : public ethercat_plugins::Technosoft_IPOS
{
  FRIEND_TEST(TechnosoftIPOSTest, CommandPosition);
  FRIEND_TEST(TechnosoftIPOSTest, CommandVelocity);
  FRIEND_TEST(TechnosoftIPOSTest, ModeOfOperation);
};

class TechnosoftIPOSTest : public ::testing::Test
{
public:
  void SetUp();
  void TearDown();

protected:
  std::unique_ptr<FriendTechnosoftIPOS> plugin_;
};

#endif  // TECHNOSOFT_PLUGINS__TEST_TECHNOSOFT_IPOS_PLUGIN_HPP_
