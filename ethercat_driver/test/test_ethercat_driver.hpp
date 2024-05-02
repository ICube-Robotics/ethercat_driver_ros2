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

#ifndef TEST_ETHERCAT_DRIVER_HPP_
#define TEST_ETHERCAT_DRIVER_HPP_

#include <gmock/gmock.h>

#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_lifecycle/state.hpp"

class TestEthercatDriver : public ::testing::Test
{
public:
  void test_ethercat_driver(std::string & urdf);

protected:
  void SetUp() override
  {
    // REMOVE THIS MEMBER ONCE FAKE COMPONENTS ARE REMOVED
    urdf_config_ =
      R"(
        <ros2_control name="EthercatSystem" type="system">
          <hardware>
            <plugin>ethercat_driver/EthercatDriver</plugin>
            <param name="master_plugin">ethercat_master/MockMaster</param>
            <param name="master_id">0</param>
            <param name="control_frequency">100</param>
          </hardware>
          <joint name="joint1">
            <command_interface name="position"/>
            <state_interface name="position"/>
            <state_interface name="velocity"/>
            <ec_module name="ECModule">
              <plugin>ethercat_generic_plugins/GenericEcSlave</plugin>
              <param name="alias">0</param>
              <param name="position">1</param>
            </ec_module>
          </joint>
          <gpio name="gpio1">
            <command_interface name="output1"/>
            <command_interface name="output2"/>
            <state_interface name="input1"/>
            <state_interface name="input2"/>
            <ec_module name="ECModule">
              <plugin>ethercat_generic_plugins/GenericEcSlave</plugin>
              <param name="alias">0</param>
              <param name="position">2</param>
            </ec_module>
          </gpio>
          <sensor name="sensor1">
            <state_interface name="input1"/>
            <state_interface name="input2"/>
            <ec_module name="ECModule">
              <plugin>ethercat_generic_plugins/GenericEcSlave</plugin>
              <param name="alias">0</param>
              <param name="position">3</param>
            </ec_module>
          </sensor>
        </ros2_control>
      )";
  }

  std::string urdf_config_;
};

class TestableResourceManager : public hardware_interface::ResourceManager
{
public:
  friend TestEthercatDriver;

  FRIEND_TEST(TestEthercatDriver, initialize_ethercat_driver);

  TestableResourceManager()
  : hardware_interface::ResourceManager() {}

  TestableResourceManager(
    const std::string & urdf, bool validate_interfaces = true, bool activate_all = false)
  : hardware_interface::ResourceManager(urdf, validate_interfaces, activate_all)
  {
  }
};

void set_components_state(
  TestableResourceManager & rm, const std::vector<std::string> & components, const uint8_t state_id,
  const std::string & state_name)
{
  for (const auto & component : components) {
    rclcpp_lifecycle::State state(state_id, state_name);
    rm.set_component_state(component, state);
  }
}

auto configure_components = [](
  TestableResourceManager & rm,
  const std::vector<std::string> & components = {"EthercatSystem"})
  {
    set_components_state(
      rm, components, lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      hardware_interface::lifecycle_state_names::INACTIVE);
  };

auto activate_components = [](
  TestableResourceManager & rm,
  const std::vector<std::string> & components = {"EthercatSystem"})
  {
    set_components_state(
      rm, components, lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      hardware_interface::lifecycle_state_names::ACTIVE);
  };

auto deactivate_components = [](
  TestableResourceManager & rm,
  const std::vector<std::string> & components = {"EthercatSystem"})
  {
    set_components_state(
      rm, components, lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      hardware_interface::lifecycle_state_names::INACTIVE);
  };

#endif  // TEST_ETHERCAT_DRIVER_HPP_
