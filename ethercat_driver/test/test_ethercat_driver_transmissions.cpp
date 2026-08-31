// Copyright 2026 ICUBE Laboratory, University of Strasbourg
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

#include <string>

#include "ethercat_driver/ethercat_driver.hpp"
#include "hardware_interface/component_parser.hpp"

namespace
{

// Promotes the protected members/methods needed to exercise the transmission machinery
// without touching bus_manager_/live hardware. Production access specifiers are unchanged.
class TestableEthercatDriver : public ethercat_driver::EthercatDriver
{
public:
  using EthercatDriver::hw_joint_states_;
  using EthercatDriver::hw_joint_commands_;
  using EthercatDriver::resizeIoBuffers;
  using EthercatDriver::loadTransmissions;
  using EthercatDriver::propagateTransmissionStates;
  using EthercatDriver::applyTransmissionCommands;
};

// Minimal kinematic tree so the URDF parser accepts the document: a base link plus one link
// per joint declared below, connected by continuous joints (no <limit> required).
const char kKinematicTree[] =
  "<link name=\"base_link\"/>"
  "<joint name=\"motor_joint\" type=\"continuous\">"
  "<parent link=\"base_link\"/><child link=\"motor_link\"/><axis xyz=\"0 0 1\"/>"
  "</joint>"
  "<link name=\"motor_link\"/>"
  "<joint name=\"output_joint\" type=\"continuous\">"
  "<parent link=\"motor_link\"/><child link=\"output_link\"/><axis xyz=\"0 0 1\"/>"
  "</joint>"
  "<link name=\"output_link\"/>";

// Builds a synthetic URDF with one drive-backed "motor_joint" (has an <ec_module>, never
// actually pluginlib-resolved by the on_init()-bypassing tests below) and one
// transmission-only "output_joint" (no <ec_module>), linked by a SimpleTransmission with the
// given reduction. `extra_motor_command_interface`, if non-empty, adds a second command
// interface to motor_joint that the transmission never declares - used to exercise the
// hold-on-unrecognized-interface behavior.
std::string makeUrdf(double reduction, const std::string & extra_motor_command_interface = "")
{
  std::string extra_interface;
  if (!extra_motor_command_interface.empty()) {
    extra_interface = "<command_interface name=\"" + extra_motor_command_interface + "\"/>";
  }
  return
    std::string("<?xml version=\"1.0\"?><robot name=\"test\">") + kKinematicTree +
    "<ros2_control name=\"test_system\" type=\"system\">"
    "<hardware><plugin>ethercat_driver/EthercatDriver</plugin></hardware>"
    "<joint name=\"motor_joint\">"
    "<state_interface name=\"position\"/>"
    "<command_interface name=\"position\"/>" +
    extra_interface +
    "<ec_module name=\"mod1\"><plugin>test_vendor/FakeDrive</plugin></ec_module>"
    "</joint>"
    "<joint name=\"output_joint\">"
    "<state_interface name=\"position\"/>"
    "<command_interface name=\"position\"/>"
    "</joint>"
    "<transmission name=\"tx1\">"
    "<plugin>transmission_interface/SimpleTransmission</plugin>"
    "<actuator name=\"motor_joint\" role=\"actuator1\"/>"
    "<joint name=\"output_joint\" role=\"joint1\">"
    "<mechanical_reduction>" + std::to_string(reduction) + "</mechanical_reduction>"
    "</joint>"
    "</transmission>"
    "</ros2_control>"
    "</robot>";
}

std::string makeUrdfMissingEcModule()
{
  return
    std::string("<?xml version=\"1.0\"?><robot name=\"test\">") + kKinematicTree +
    "<ros2_control name=\"test_system\" type=\"system\">"
    "<hardware><plugin>ethercat_driver/EthercatDriver</plugin></hardware>"
    "<joint name=\"motor_joint\">"
    "<state_interface name=\"position\"/>"
    "<command_interface name=\"position\"/>"
    "</joint>"
    "<joint name=\"output_joint\">"
    "<state_interface name=\"position\"/>"
    "<command_interface name=\"position\"/>"
    "</joint>"
    "<transmission name=\"tx1\">"
    "<plugin>transmission_interface/SimpleTransmission</plugin>"
    "<actuator name=\"motor_joint\" role=\"actuator1\"/>"
    "<joint name=\"output_joint\" role=\"joint1\"/>"
    "</transmission>"
    "</ros2_control>"
    "</robot>";
}

std::string makeUrdfUnknownPlugin()
{
  return
    std::string("<?xml version=\"1.0\"?><robot name=\"test\">") + kKinematicTree +
    "<ros2_control name=\"test_system\" type=\"system\">"
    "<hardware><plugin>ethercat_driver/EthercatDriver</plugin></hardware>"
    "<joint name=\"motor_joint\">"
    "<state_interface name=\"position\"/>"
    "<command_interface name=\"position\"/>"
    "<ec_module name=\"mod1\"><plugin>test_vendor/FakeDrive</plugin></ec_module>"
    "</joint>"
    "<joint name=\"output_joint\">"
    "<state_interface name=\"position\"/>"
    "<command_interface name=\"position\"/>"
    "</joint>"
    "<transmission name=\"tx1\">"
    "<plugin>nonexistent_vendor/NoSuchTransmission</plugin>"
    "<actuator name=\"motor_joint\" role=\"actuator1\"/>"
    "<joint name=\"output_joint\" role=\"joint1\"/>"
    "</transmission>"
    "</ros2_control>"
    "</robot>";
}

// A transmission <joint> naming a joint that doesn't exist in the component at all - the
// ros2_control URDF parser itself rejects this while building HardwareInfo, before
// loadTransmissions() ever runs.
std::string makeUrdfUnknownJointInComponent()
{
  return
    std::string("<?xml version=\"1.0\"?><robot name=\"test\">") + kKinematicTree +
    "<ros2_control name=\"test_system\" type=\"system\">"
    "<hardware><plugin>ethercat_driver/EthercatDriver</plugin></hardware>"
    "<joint name=\"motor_joint\">"
    "<state_interface name=\"position\"/>"
    "<command_interface name=\"position\"/>"
    "<ec_module name=\"mod1\"><plugin>test_vendor/FakeDrive</plugin></ec_module>"
    "</joint>"
    "<transmission name=\"tx1\">"
    "<plugin>transmission_interface/SimpleTransmission</plugin>"
    "<actuator name=\"motor_joint\" role=\"actuator1\"/>"
    "<joint name=\"nonexistent_joint\" role=\"joint1\"/>"
    "</transmission>"
    "</ros2_control>"
    "</robot>";
}

// Parses the URDF and runs the base SystemInterface::on_init() + IO buffer sizing - exactly
// the subset loadTransmissions() needs. Explicitly qualified to bypass
// EthercatDriver::on_init()'s virtual override, which would additionally require
// bus_manager_.configureModules() / pluginlib-loadable EC slaves. Returns false (instead of
// letting an exception escape) on any parse/init failure, since callers only care whether
// initialization succeeded, not which layer rejected a malformed URDF.
bool initFromUrdf(TestableEthercatDriver & driver, const std::string & urdf)
{
  try {
    auto infos = hardware_interface::parse_control_resources_from_urdf(urdf);
    if (infos.empty()) {return false;}
    hardware_interface::HardwareComponentInterfaceParams params;
    params.hardware_info = infos.at(0);
    if (driver.hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
      return false;
    }
  } catch (const std::exception &) {
    return false;
  }
  driver.resizeIoBuffers();
  return true;
}

}  // namespace

TEST(TestEthercatDriverTransmissions, ReadDirectionAppliesReduction)
{
  TestableEthercatDriver driver;
  ASSERT_TRUE(initFromUrdf(driver, makeUrdf(2.0)));
  ASSERT_TRUE(driver.loadTransmissions());

  // motor_joint (index 0) declares only "position", so its slot 0 is position.
  driver.hw_joint_states_[0][0] = 4.0;
  driver.propagateTransmissionStates();

  EXPECT_DOUBLE_EQ(driver.hw_joint_states_[1][0], 2.0);
}

TEST(TestEthercatDriverTransmissions, WriteDirectionAppliesReduction)
{
  TestableEthercatDriver driver;
  ASSERT_TRUE(initFromUrdf(driver, makeUrdf(2.0)));
  ASSERT_TRUE(driver.loadTransmissions());

  driver.hw_joint_commands_[1][0] = 3.0;
  driver.applyTransmissionCommands();

  EXPECT_DOUBLE_EQ(driver.hw_joint_commands_[0][0], 6.0);
}

TEST(TestEthercatDriverTransmissions, ActuatorWithoutEcModuleFails)
{
  TestableEthercatDriver driver;
  ASSERT_TRUE(initFromUrdf(driver, makeUrdfMissingEcModule()));
  EXPECT_FALSE(driver.loadTransmissions());
}

TEST(TestEthercatDriverTransmissions, UnknownJointNameFails)
{
  TestableEthercatDriver driver;
  EXPECT_FALSE(initFromUrdf(driver, makeUrdfUnknownJointInComponent()));
}

TEST(TestEthercatDriverTransmissions, UnknownPluginTypeFails)
{
  TestableEthercatDriver driver;
  ASSERT_TRUE(initFromUrdf(driver, makeUrdfUnknownPlugin()));
  EXPECT_FALSE(driver.loadTransmissions());
}

TEST(TestEthercatDriverTransmissions, UnrecognizedActuatorCommandHoldsPreviousValue)
{
  TestableEthercatDriver driver;
  ASSERT_TRUE(initFromUrdf(driver, makeUrdf(2.0, "reset_fault")));
  ASSERT_TRUE(driver.loadTransmissions());

  // reset_fault is index 1 on motor_joint (after position); never declared on the
  // transmission's joint side, so SimpleTransmission never touches it.
  driver.hw_joint_commands_[0][1] = 1.0;
  driver.applyTransmissionCommands();

  EXPECT_DOUBLE_EQ(driver.hw_joint_commands_[0][1], 1.0);
}
