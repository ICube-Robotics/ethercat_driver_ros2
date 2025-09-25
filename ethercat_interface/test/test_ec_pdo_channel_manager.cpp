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
#include <string>
#include <vector>
#include <iostream>
#include <bitset>

#include "ethercat_interface/ec_pdo_single_interface_channel_manager.hpp"
#include "ethercat_interface/ec_pdo_group_interface_channel_manager.hpp"
#include "yaml-cpp/yaml.h"

TEST(TestEcPdoSingleInterfaceChannelManager, LoadFromConfig)
{
  const char channel_config[] =
    R"(
      {index: 0x6071, sub_index: 0, type: int16, command_interface: effort, default: -5, factor: 2, offset: 10}
    )";
  YAML::Node config = YAML::Load(channel_config);
  ethercat_interface::EcPdoSingleInterfaceChannelManager pdo_manager;
  pdo_manager.pdo_type = ethercat_interface::PdoType::RPDO;
  pdo_manager.load_from_config(config);

  ASSERT_EQ(pdo_manager.index, 0x6071);
  ASSERT_EQ(pdo_manager.sub_index, 0);
  ASSERT_EQ(pdo_manager.data_type(), "int16");
  ASSERT_EQ(pdo_manager.interface_name(), "effort");
  ASSERT_EQ(pdo_manager.default_value, -5);
  ASSERT_EQ(pdo_manager.factor, 2);
  ASSERT_EQ(pdo_manager.offset, 10);
}

TEST(TestEcPdoSingleInterfaceChannelManager, EcReadS16)
{
  const char channel_config[] =
    R"(
      {index: 0x6071, sub_index: 0, type: int16, command_interface: effort, default: -5, factor: 2, offset: 10}
    )";
  YAML::Node config = YAML::Load(channel_config);
  ethercat_interface::EcPdoSingleInterfaceChannelManager pdo_manager;
  pdo_manager.pdo_type = ethercat_interface::PdoType::RPDO;
  pdo_manager.load_from_config(config);

  uint8_t buffer[16];
  EC_WRITE_S16(buffer, 42);
  ASSERT_EQ(pdo_manager.ec_read(buffer), 2 * 42 + 10);
}

TEST(TestEcPdoSingleInterfaceChannelManager, EcReadWriteBit2)
{
  const char channel_config[] =
    R"(
      {index: 0x6071, sub_index: 0, type: bit2, mask: 3}
    )";
  YAML::Node config = YAML::Load(channel_config);
  ethercat_interface::EcPdoSingleInterfaceChannelManager pdo_manager;
  pdo_manager.pdo_type = ethercat_interface::PdoType::RPDO;
  pdo_manager.load_from_config(config);

  ASSERT_EQ(pdo_manager.data_type(), "bit2");
  ASSERT_EQ(pdo_manager.mask, 3);
  ASSERT_EQ(ethercat_interface::type2bits(pdo_manager.data_type()), 2);

  uint8_t buffer[1];
  EC_WRITE_U8(buffer, 0);
  ASSERT_EQ(pdo_manager.ec_read(buffer), 0);
  EC_WRITE_U8(buffer, 3);
  ASSERT_EQ(pdo_manager.ec_read(buffer), 3);
  EC_WRITE_U8(buffer, 5);
  ASSERT_EQ(pdo_manager.ec_read(buffer), 1);

  pdo_manager.ec_write(buffer, 0);
  ASSERT_EQ(EC_READ_U8(buffer), 4);
  pdo_manager.ec_write(buffer, 2);
  ASSERT_EQ(EC_READ_U8(buffer), 6);
  EC_WRITE_U8(buffer, 0);
  pdo_manager.ec_write(buffer, 5);
  ASSERT_EQ(EC_READ_U8(buffer), 1);
}

TEST(TestEcPdoSingleInterfaceChannelManager, EcReadWriteBoolMask1)
{
  const char channel_config[] =
    R"(
      {index: 0x6071, sub_index: 0, type: bool, mask: 1}
    )";
  YAML::Node config = YAML::Load(channel_config);
  ethercat_interface::EcPdoSingleInterfaceChannelManager pdo_manager;
  pdo_manager.pdo_type = ethercat_interface::PdoType::RPDO;
  pdo_manager.load_from_config(config);

  ASSERT_EQ(pdo_manager.data_type(), "bool");
  ASSERT_EQ(pdo_manager.mask, 1);
  ASSERT_EQ(ethercat_interface::type2bits(pdo_manager.data_type()), 1);

  uint8_t buffer[1];
  EC_WRITE_U8(buffer, 3);
  ASSERT_EQ(pdo_manager.ec_read(buffer), 1);
  EC_WRITE_U8(buffer, 0);
  ASSERT_EQ(pdo_manager.ec_read(buffer), 0);

  pdo_manager.ec_write(buffer, 0);
  ASSERT_EQ(EC_READ_U8(buffer), 0);
  pdo_manager.ec_write(buffer, 5);
  ASSERT_EQ(EC_READ_U8(buffer), 1);
}

TEST(TestEcPdoSingleInterfaceChannelManager, EcReadWriteBit8Mask5)
{
  const char channel_config[] =
    R"(
      {index: 0x6071, sub_index: 0, type: bit8, mask: 5}
    )";
  YAML::Node config = YAML::Load(channel_config);
  ethercat_interface::EcPdoSingleInterfaceChannelManager pdo_manager;
  pdo_manager.pdo_type = ethercat_interface::PdoType::RPDO;
  pdo_manager.load_from_config(config);

  ASSERT_EQ(pdo_manager.data_type(), "bit8");
  ASSERT_EQ(pdo_manager.mask, 5);  // < Set mask 0b00000101
  ASSERT_EQ(ethercat_interface::type2bits(pdo_manager.data_type()), 8);

  uint8_t buffer[1];
  // Should only soft read the bit 5 and 1 that is both in the mask and in the buffer
  EC_WRITE_U8(buffer, 7);  // < Hard write 0b00000111
  ASSERT_EQ(pdo_manager.ec_read(buffer), 5);

  // Hard write 0, should soft read 0
  EC_WRITE_U8(buffer, 0);
  ASSERT_EQ(pdo_manager.ec_read(buffer), 0);

  // Soft write 0, should hard read 0
  pdo_manager.ec_write(buffer, 0);
  ASSERT_EQ(EC_READ_U8(buffer), 0);

  // Soft write 3 (with mask applied is 1) should hard read 0b00000001
  pdo_manager.ec_write(buffer, 3);
  ASSERT_EQ(EC_READ_U8(buffer), 1);

  // Soft write 7 (with mask applied is 5) should hard read 0b00000101
  pdo_manager.ec_write(buffer, 7);
  ASSERT_EQ(EC_READ_U8(buffer), 5);

  // Soft write 5 (with mask applied is 5) should hard read 0b00000101
  pdo_manager.ec_write(buffer, 5);
  ASSERT_EQ(EC_READ_U8(buffer), 5);
}


// This test is very weird, the expected behaviour is somehow confusing
// since it writes the entire octet, but read only the bits set to 1 in the mask
// and converts the result to 1 or 0 (1 if at least one bit is set to 1)
TEST(TestEcPdoSingleInterfaceChannelManager, EcReadWriteBoolMask5)
{
  const char channel_config[] =
    R"(
      {index: 0x6071, sub_index: 0, type: bool, mask: 5}
    )";
  YAML::Node config = YAML::Load(channel_config);
  ethercat_interface::EcPdoSingleInterfaceChannelManager pdo_manager;
  pdo_manager.pdo_type = ethercat_interface::PdoType::RPDO;
  ASSERT_EQ(pdo_manager.load_from_config(config), false);

  return;
  // ASSERT_EQ(pdo_manager.data_type(), "bool");
  // ASSERT_EQ(pdo_manager.mask, 5);  // < Set mask 0b00000101
  // ASSERT_EQ(ethercat_interface::type2bits(pdo_manager.data_type()), 1);

  // uint8_t buffer[1];
  // // Should only soft read the bit 1 that is both in the mask and in the buffer
  // EC_WRITE_U8(buffer, 7);  // < Hard write 0b00000111
  // ASSERT_EQ(pdo_manager.ec_read(buffer), 1);

  // // Hard write 0, should soft read 0
  // EC_WRITE_U8(buffer, 0);
  // ASSERT_EQ(pdo_manager.ec_read(buffer), 0);

  // // Soft write 0, should hard read 0
  // pdo_manager.ec_write(buffer, 0);
  // ASSERT_EQ(EC_READ_U8(buffer), 0);

  // // Soft write 3 (with mask applied is 1) should hard read 0b00000001
  // pdo_manager.ec_write(buffer, 3);
  // ASSERT_EQ(EC_READ_U8(buffer), 1);

  // // Soft write 7 (with mask applied is 5) should hard read 0b00000101
  // pdo_manager.ec_write(buffer, 7);
  // ASSERT_EQ(EC_READ_U8(buffer), 5);

  // // Soft write 5 (with mask applied is 5) should hard read 0b00000101
  // pdo_manager.ec_write(buffer, 5);
  // ASSERT_EQ(EC_READ_U8(buffer), 5);
}


TEST(TestEcPdoGroupInterfaceChannelManager, LoadConfigTest)
{
  const char channel_config[] =
    R"(
      {
        index: 0xf788,
        sub_index: 0x00,
        type: bit240,
        data_mapping: [
          {
            addr_offset: 60,
            type: int32,
            factor: 3.14,
            offset: 2.71,
            command_interface: effort
          },
          {
            addr_offset: 64,
            type: int16,
            factor: 1.1,
            offset: 0.1,
            state_interface: position
          },
          {
            addr_offset: 66,
            type: uint8,
            mask: 7,
          },
          {
            addr_offset: 67,
            type: bool,
            mask: 8,
          }
        ]
      }
    )";
  YAML::Node config = YAML::Load(channel_config);
  ethercat_interface::EcPdoGroupInterfaceChannelManager pdo_manager;
  pdo_manager.pdo_type = ethercat_interface::PdoType::RPDO;
  ASSERT_EQ(pdo_manager.load_from_config(config), true);

  ASSERT_EQ(pdo_manager.number_of_interfaces(), 5);
  ASSERT_EQ(pdo_manager.number_of_managed_interfaces(), 2);

  ASSERT_EQ(pdo_manager.data_type(0), "bit240");
  ASSERT_EQ(pdo_manager.interface_name(0), "null");
  ASSERT_EQ(pdo_manager.index, 0xf788);
  ASSERT_EQ(pdo_manager.sub_index, 0);

  ASSERT_EQ(pdo_manager.data_type(1), "int32");
  ASSERT_EQ(pdo_manager.interface_name(1), "effort");
  ASSERT_EQ(pdo_manager.data(1).factor, 3.14);
  ASSERT_EQ(pdo_manager.data(1).offset, 2.71);
  ASSERT_EQ(pdo_manager.v_data[1].addr_offset, 60);

  ASSERT_EQ(pdo_manager.data_type(2), "int16");
  ASSERT_EQ(pdo_manager.interface_name(2), "position");
  ASSERT_EQ(pdo_manager.data(2).factor, 1.1);
  ASSERT_EQ(pdo_manager.data(2).offset, 0.1);
  ASSERT_EQ(pdo_manager.v_data[2].addr_offset, 64);

  ASSERT_EQ(pdo_manager.data_type(3), "uint8");
  ASSERT_EQ(pdo_manager.interface_name(3), "null");
  ASSERT_EQ(pdo_manager.data(3).mask, 7);
  ASSERT_EQ(pdo_manager.v_data[3].addr_offset, 66);

  ASSERT_EQ(pdo_manager.data_type(4), "bool");
  ASSERT_EQ(pdo_manager.interface_name(4), "null");
  ASSERT_EQ(pdo_manager.data(4).mask, 8);
}


TEST(TestEcPdoGroupInterfaceChannelManager, ReadWriteBits)
{
  const char channel_config[] =
    R"(
      {
        index: 0x6071,
        sub_index: 0x00,
        type: bit8,
        data_mapping: [
          {
            type: bool,
            mask: 1,
            command_interface: input1
          },
          {
            type: bool,
            mask: 2,
            state_interface: output1
          },
          {
            type: bool,
            mask: 4,
            command_interface: input2
          },
          {
            type: bool,
            mask: 8,
            state_interface: output2
          },
          {
            type: bool,
            mask: 16,
            command_interface: input3
          },
          {
            type: bool,
            mask: 32,
            state_interface: output3
          }
        ]
      }
    )";
  YAML::Node config = YAML::Load(channel_config);
  ethercat_interface::EcPdoGroupInterfaceChannelManager pdo_manager;
  pdo_manager.pdo_type = ethercat_interface::PdoType::RPDO;
  ASSERT_EQ(pdo_manager.load_from_config(config), true);

  ASSERT_EQ(pdo_manager.number_of_interfaces(), 7);
  ASSERT_EQ(pdo_manager.number_of_managed_interfaces(), 6);

  ASSERT_EQ(pdo_manager.data_type(0), "bit8");
  ASSERT_EQ(pdo_manager.interface_name(0), "null");
  ASSERT_EQ(pdo_manager.index, 0x6071);
  ASSERT_EQ(pdo_manager.sub_index, 0);

  ASSERT_EQ(pdo_manager.data_type(1), "bool");
  ASSERT_EQ(pdo_manager.interface_name(1), "input1");
  ASSERT_EQ(pdo_manager.data(1).mask, 1);
  ASSERT_EQ(pdo_manager.v_data[1].addr_offset, 0);

  ASSERT_EQ(pdo_manager.data_type(2), "bool");
  ASSERT_EQ(pdo_manager.interface_name(2), "output1");
  ASSERT_EQ(pdo_manager.data(2).mask, 2);
  ASSERT_EQ(pdo_manager.v_data[2].addr_offset, 0);

  ASSERT_EQ(pdo_manager.data_type(3), "bool");
  ASSERT_EQ(pdo_manager.interface_name(3), "input2");
  ASSERT_EQ(pdo_manager.data(3).mask, 4);
  ASSERT_EQ(pdo_manager.v_data[3].addr_offset, 0);

  ASSERT_EQ(pdo_manager.data_type(4), "bool");
  ASSERT_EQ(pdo_manager.interface_name(4), "output2");
  ASSERT_EQ(pdo_manager.data(4).mask, 8);
  ASSERT_EQ(pdo_manager.v_data[4].addr_offset, 0);

  ASSERT_EQ(pdo_manager.data_type(5), "bool");
  ASSERT_EQ(pdo_manager.interface_name(5), "input3");
  ASSERT_EQ(pdo_manager.data(5).mask, 16);
  ASSERT_EQ(pdo_manager.v_data[5].addr_offset, 0);

  ASSERT_EQ(pdo_manager.data_type(6), "bool");
  ASSERT_EQ(pdo_manager.interface_name(6), "output3");
  ASSERT_EQ(pdo_manager.data(6).mask, 32);
  ASSERT_EQ(pdo_manager.v_data[6].addr_offset, 0);

  uint8_t buffer[1];
  std::vector<uint8_t> write_tests0 =
  {
    0, 1, 2, 4, 8, 16, 32,
    0b00111111,
    0b11000000,
    0b11111111,
    0b00101010,
    0b11010101,
    0b00001111
  };

  for (size_t n = 0; n < write_tests0.size(); ++n) {
    EC_WRITE_U8(buffer, write_tests0[n]);
    ASSERT_EQ(pdo_manager.ec_read(buffer), write_tests0[n]);

    std::bitset<8> bits(write_tests0[n]);
    for (size_t i = 1; i < 7; i++) {
      ASSERT_EQ(pdo_manager.ec_read(buffer, i), bits.test(i - 1));
    }
  }
}
