// Copyright 2024 ICUBE Laboratory, University of Strasbourg
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
//
// Author: Manuel YGUEL (yguel.robotics@gmail.com)

// Executes directly the gtest, call:
//   ./build/ethercat_driver/test_ethercat_safety_driver
// from the root directory of the workspace

#include <gtest/gtest.h>
#include <fstream>
#include <sstream>
#include <filesystem>

#include "testHelper_ethercat_safety_driver.hpp"

TEST(TestEthercatSafetyDriver, getEcTransferModuleParam)
{
  ethercat_driver::TestHelperEthercatSafetyDriver driver;
  std::filesystem::path dir = TEST_RESOURCES_DIRECTORY;
  const std::string test_config_path = dir / "test_config_ethercat_safety.yaml";

  std::cout << "Test config path: " << test_config_path << std::endl;

  YAML::Node config = YAML::LoadFile(test_config_path);

  auto modules = driver.getEcTransferModuleParam(config);
  std::set<std::string> names;
  for (auto & module : modules) {
    auto it = module.find("name");
    EXPECT_TRUE(
      it !=
      nullptr) << "name is not a field of the ec safety module" << std::endl;
    names.insert(it->second);
  }
  std::set<std::string> expected_names = {
    "s2", "m1", "m2"
  };
  EXPECT_EQ(names, expected_names) << "Ec module names are not as expected" << std::endl;
}

TEST(TestEthercatSafetyDriver, getEcTransferNet)
{
  ethercat_driver::TestHelperEthercatSafetyDriver driver;
  std::string yaml;
  {
    std::filesystem::path dir = TEST_RESOURCES_DIRECTORY;
    const std::string test_config_path = dir / "test_config_ethercat_safety.yaml";
    std::ifstream file(test_config_path);
    std::stringstream buffer;
    buffer << file.rdbuf();
    yaml = buffer.str();
  }

  YAML::Node config = YAML::Load(yaml);

  auto nets = driver.getEcTransferNets(config);
  std::set<std::string> net_names;

  EXPECT_EQ(nets.size(), 2) << "Number of safety nets is not as expected" << std::endl;
  for (auto & net : nets) {
    net_names.insert(net.name);
  }
  std::set<std::string> expected_net_names = {
    "n1", "n2"
  };
  EXPECT_EQ(net_names, expected_net_names) << "Net names are not as expected" << std::endl;

  int net_1_index = nets[0].name == "n1" ? 0 : 1;
  int net_2_index = (net_1_index + 1) % 2;

  auto & net1 = nets[net_1_index];
  auto & net2 = nets[net_2_index];

  // Net 1
  EXPECT_EQ(net1.master, "m1") << "Master name is not as expected" << std::endl;
  EXPECT_EQ(
    net1.transfers.size(),
    2) << "Number of transfers is not as expected for net n1" << std::endl;
  {
    {
      auto & tr = net1.transfers[0];
      EXPECT_EQ(
        tr.input.module_name,
        "s1") << "Input module name is not as expected for 1st transfer of net n1" << std::endl;
      EXPECT_EQ(
        tr.input.index,
        0x607a) << "Input index is not as expected for 1st transfer of net n1" << std::endl;

      EXPECT_EQ(
        tr.output.module_name,
        "s2") << "Output module name is not as expected for 1st transfer of net n1" << std::endl;
      EXPECT_EQ(
        tr.output.index,
        0x60ff) << "Output index is not as expected for 1st transfer of net n1" << std::endl;

      EXPECT_EQ(
        tr.size,
        18) << "Size is not as expected for 1st transfer of net n1" << std::endl;
    }

    {
      auto & tr = net1.transfers[1];
      EXPECT_EQ(
        tr.input.module_name,
        "s1") << "Input module name is not as expected for 2nd transfer of net n1" << std::endl;
      EXPECT_EQ(
        tr.input.index,
        0x707a) << "Input index is not as expected for 2nd transfer of net n1" << std::endl;


      EXPECT_EQ(
        tr.output.module_name,
        "s3") << "Output module name is not as expected for 2nd transfer of net n1" << std::endl;
      EXPECT_EQ(
        tr.output.index,
        0x10ff) << "Output index is not as expected for 2nd transfer of net n1" << std::endl;

      EXPECT_EQ(
        tr.size,
        42) << "Size is not as expected for 2nd transfer of net n1" << std::endl;
    }
  }

  // Net 2
  EXPECT_EQ(net2.master, "m2") << "Master name is not as expected" << std::endl;
  EXPECT_EQ(
    net2.transfers.size(),
    1) << "Number of transfers is not as expected for net n2" << std::endl;
  {
    auto & tr = net2.transfers[0];
    EXPECT_EQ(
      tr.input.module_name,
      "q1") << "Input module name is not as expected for 1st transfer of net n2" << std::endl;
    EXPECT_EQ(
      tr.input.index,
      0x607a) << "Input index is not as expected for 1st transfer of net n2" << std::endl;

    EXPECT_EQ(
      tr.output.module_name,
      "q2") << "Output module name is not as expected for 1st transfer of net n2" << std::endl;
    EXPECT_EQ(
      tr.output.index,
      0x60ff) << "Output index is not as expected for 1st transfer of net n2" << std::endl;

    EXPECT_EQ(
      tr.size,
      18) << "Size is not as expected for 1st transfer of net n2" << std::endl;
  }
}

TEST(TestEthercatSafetyDriver, estopParseConfigFile)
{
  ethercat_driver::TestHelperEthercatSafetyDriver driver;
  std::string yaml;
  {
    std::filesystem::path dir = TEST_RESOURCES_DIRECTORY;
    const std::string test_config_path = dir / "estop_ethercat_safety.yaml";
    std::ifstream file(test_config_path);
    std::stringstream buffer;
    buffer << file.rdbuf();
    yaml = buffer.str();
  }

  YAML::Node config = YAML::Load(yaml);

  auto nets = driver.getEcTransferNets(config);
  EXPECT_EQ(1, nets.size()) << "Number of safety nets is not as expected" << std::endl;

  auto & net = nets[0];
  EXPECT_EQ(net.master, "el1918") << "Master name is not as expected" << std::endl;

  EXPECT_EQ(
    net.transfers.size(),
    2) << "Number of transfers is not as expected for net n1" << std::endl;
  {
    auto & tr = net.transfers[0];
    EXPECT_EQ(
      tr.input.module_name,
      "ek1914") << "Input module name is not as expected for 1st transfer of net n1" << std::endl;
    EXPECT_EQ(
      tr.input.index,
      0x6000) << "Input index is not as expected for 1st transfer" << std::endl;

    EXPECT_EQ(
      tr.output.module_name,
      "el1918") << "Output module name is not as expected for 1st transfer" << std::endl;
    EXPECT_EQ(
      tr.output.index,
      0x7080) << "Output index is not as expected for 1st transfer" << std::endl;

    EXPECT_EQ(
      tr.size,
      6) << "Size is not as expected for 1st transfer" << std::endl;
  }
  {
    auto & tr = net.transfers[1];
    EXPECT_EQ(
      tr.input.module_name,
      "el1918") << "Input module name is not as expected for 2nd transfer" << std::endl;
    EXPECT_EQ(
      tr.input.index,
      0x6080) << "Input index is not as expected for 2nd transfer" << std::endl;

    EXPECT_EQ(
      tr.output.module_name,
      "ek1914") << "Output module name is not as expected for 2nd transfer" << std::endl;
    EXPECT_EQ(
      tr.output.index,
      0x7000) << "Output index is not as expected for 2nd transfer" << std::endl;

    EXPECT_EQ(
      tr.size,
      6) << "Size is not as expected for 2nd transfer" << std::endl;
  }
}
