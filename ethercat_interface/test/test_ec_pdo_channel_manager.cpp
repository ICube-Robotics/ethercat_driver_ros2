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

#include <gtest/gtest.h>
#include <memory>

#include "ethercat_interface/ec_pdo_channel_manager.hpp"
#include "yaml-cpp/yaml.h"

TEST(TestEcPdoChannelManager, load_from_config)
{
  const char channel_config[] =
    R"(
      {index: 0x6071, sub_index: 0, type: int16, command_interface: effort, default: -5, factor: 2, offset: 10}
    )";
  YAML::Node config = YAML::Load(channel_config);
  ethercat_interface::EcPdoChannelManager pdo_manager;
  pdo_manager.pdo_type = ethercat_interface::PdoType::RPDO;
  pdo_manager.load_from_config(config);

  ASSERT_EQ(pdo_manager.index, 0x6071);
  ASSERT_EQ(pdo_manager.sub_index, 0);
  ASSERT_EQ(pdo_manager.data_type, "int16");
  ASSERT_EQ(pdo_manager.interface_name, "effort");
  ASSERT_EQ(pdo_manager.default_value, -5);
  ASSERT_EQ(pdo_manager.factor, 2);
  ASSERT_EQ(pdo_manager.offset, 10);
}
