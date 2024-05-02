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

#include "test_ethercat_driver.hpp"
#include "ros2_control_test_assets/components_urdfs.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

TEST_F(TestEthercatDriver, load_ethercat_driver)
{
  auto urdf = ros2_control_test_assets::urdf_head + urdf_config_ +
    ros2_control_test_assets::urdf_tail;
  ASSERT_NO_THROW(TestableResourceManager rm(urdf));
}

TEST_F(TestEthercatDriver, initialize_ethercat_driver)
{
  auto urdf = ros2_control_test_assets::urdf_head + urdf_config_ +
    ros2_control_test_assets::urdf_tail;
  TestableResourceManager rm(urdf);

  // check is hardware is configured
  auto status_map = rm.get_components_status();
  EXPECT_EQ(
    status_map["EthercatSystem"].state.label(),
    hardware_interface::lifecycle_state_names::UNCONFIGURED);
  configure_components(rm);
  status_map = rm.get_components_status();
  EXPECT_EQ(
    status_map["EthercatSystem"].state.label(),
    hardware_interface::lifecycle_state_names::INACTIVE);
  activate_components(rm);
  status_map = rm.get_components_status();
  EXPECT_EQ(
    status_map["EthercatSystem"].state.label(),
    hardware_interface::lifecycle_state_names::ACTIVE);

  // Check interfaces
  EXPECT_EQ(1u, rm.system_components_size());
  ASSERT_EQ(6u, rm.state_interface_keys().size());
  EXPECT_TRUE(rm.state_interface_exists("joint1/position"));
  EXPECT_TRUE(rm.state_interface_exists("joint1/velocity"));
  EXPECT_TRUE(rm.state_interface_exists("gpio1/input1"));
  EXPECT_TRUE(rm.state_interface_exists("gpio1/input2"));
  EXPECT_TRUE(rm.state_interface_exists("sensor1/input1"));
  EXPECT_TRUE(rm.state_interface_exists("sensor1/input2"));

  ASSERT_EQ(3u, rm.command_interface_keys().size());
  EXPECT_TRUE(rm.command_interface_exists("joint1/position"));
  EXPECT_TRUE(rm.command_interface_exists("gpio1/output1"));
  EXPECT_TRUE(rm.command_interface_exists("gpio1/output2"));

  // Check initial values
  hardware_interface::LoanedStateInterface joint1_si_position =
    rm.claim_state_interface("joint1/position");
  hardware_interface::LoanedStateInterface gpio1_si_input2 =
    rm.claim_state_interface("gpio1/input2");
  hardware_interface::LoanedStateInterface sensor1_si_input1 =
    rm.claim_state_interface("sensor1/input1");
  hardware_interface::LoanedCommandInterface gpio1_ci_output1 =
    rm.claim_command_interface("gpio1/output1");

  // State interfaces without initial value are set to NaN
  ASSERT_TRUE(std::isnan(joint1_si_position.get_value()));
  ASSERT_TRUE(std::isnan(gpio1_si_input2.get_value()));
  ASSERT_TRUE(std::isnan(sensor1_si_input1.get_value()));
  ASSERT_TRUE(std::isnan(gpio1_ci_output1.get_value()));

  // set some new values in commands
  gpio1_ci_output1.set_value(0.123);

  // State values should not be changed
  ASSERT_TRUE(std::isnan(gpio1_si_input2.get_value()));
  ASSERT_EQ(0.123, gpio1_ci_output1.get_value());
}
