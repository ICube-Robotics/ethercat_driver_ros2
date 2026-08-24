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
#include <cstdint>
#include <cstring>

#include "ethercat_interface/ec_sdo_manager.hpp"
#include "yaml-cpp/yaml.h"

TEST(TestEcSdoManager, LoadFromConfigUint32)
{
  const char sdo_config[] =
    R"(
      {index: 0x6076, sub_index: 0, type: uint32, value: 3000}
    )";
  YAML::Node config = YAML::Load(sdo_config);
  ethercat_interface::SdoConfigEntry sdo_entry;
  ASSERT_TRUE(sdo_entry.load_from_config(config));

  ASSERT_EQ(sdo_entry.index, 0x6076);
  ASSERT_EQ(sdo_entry.data_type, "uint32");

  uint8_t buffer[4];
  sdo_entry.buffer_write(buffer);
  ASSERT_EQ(EC_READ_U32(buffer), 3000u);
}

TEST(TestEcSdoManager, LoadFromConfigHexValue)
{
  // Every drive config in the repo clears faults via `{..., type: uint16, value: 0x80}` on
  // 0x6040 -- yaml-cpp's as<double>() does not parse hex literals, only as<int64_t>()/as<int>()
  // do, so this guards against a fix for fractional values silently breaking every hex one.
  const char sdo_config[] =
    R"(
      {index: 0x6040, sub_index: 0, type: uint16, value: 0x80}
    )";
  YAML::Node config = YAML::Load(sdo_config);
  ethercat_interface::SdoConfigEntry sdo_entry;
  ASSERT_TRUE(sdo_entry.load_from_config(config));

  uint8_t buffer[2];
  sdo_entry.buffer_write(buffer);
  ASSERT_EQ(EC_READ_U16(buffer), 0x80u);
}

TEST(TestEcSdoManager, LoadFromConfigFloat32Fractional)
{
  const char sdo_config[] =
    R"(
      {index: 0x209A, sub_index: 0, type: float32, value: 2.5}
    )";
  YAML::Node config = YAML::Load(sdo_config);
  ethercat_interface::SdoConfigEntry sdo_entry;
  ASSERT_TRUE(sdo_entry.load_from_config(config));

  uint8_t buffer[4];
  sdo_entry.buffer_write(buffer);

  const uint32_t bits = EC_READ_U32(buffer);
  float written = 0.0f;
  std::memcpy(&written, &bits, sizeof(written));
  ASSERT_FLOAT_EQ(written, 2.5f);
}

TEST(TestEcSdoManager, LoadFromConfigReal32AliasFractional)
{
  const char sdo_config[] =
    R"(
      {index: 0x2100, sub_index: 0, type: real32, value: 9.85}
    )";
  YAML::Node config = YAML::Load(sdo_config);
  ethercat_interface::SdoConfigEntry sdo_entry;
  ASSERT_TRUE(sdo_entry.load_from_config(config));

  uint8_t buffer[4];
  sdo_entry.buffer_write(buffer);

  const uint32_t bits = EC_READ_U32(buffer);
  float written = 0.0f;
  std::memcpy(&written, &bits, sizeof(written));
  ASSERT_FLOAT_EQ(written, 9.85f);
}
