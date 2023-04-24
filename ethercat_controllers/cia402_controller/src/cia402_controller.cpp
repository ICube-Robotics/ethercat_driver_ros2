// Copyright 2023, ICube Laboratory, University of Strasbourg
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

#include "ethercat_controllers/cia402_controller.hpp"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace ethercat_controllers
{
using hardware_interface::LoanedCommandInterface;

CiA402Controller::CiA402Controller()
: controller_interface::ControllerInterface(),
  rt_command_ptr_(nullptr),
  joints_command_subscriber_(nullptr)
{
}

CallbackReturn CiA402Controller::on_init()
{
  try {
    // definition of the parameters that need to be queried from the
    // controller configuration file with default values
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn CiA402Controller::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // getting the names of the joints to be controlled
  joint_names_ = get_node()->get_parameter("joints").as_string_array();

  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
    return CallbackReturn::FAILURE;
  }

  // the desired reference trajectory is queried from the reference topic
  // and passed to update via a rt pipe
  joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
    "~/reference", rclcpp::SystemDefaultsQoS(),
    [this](const CmdType::SharedPtr msg) {rt_command_ptr_.writeFromNonRT(msg);});

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}
// As impedance control targets the effort interface, it can be directly defined here
// without the need of getting as parameter. The effort interface is then affected to
// all controlled joints.
controller_interface::InterfaceConfiguration
CiA402Controller::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(joint_names_.size());
  for (const auto & joint_name : joint_names_) {
    conf.names.push_back(joint_name + "/" + "acceleration");
  }
  return conf;
}
// Impedance control requires both velocity and position states. For this reason
// there can be directly defined here without the need of getting as parameters.
// The state interfaces are then deployed to all targeted joints.
controller_interface::InterfaceConfiguration
CiA402Controller::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(joint_names_.size() * 2);
  for (const auto & joint_name : joint_names_) {
    conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
    conf.names.push_back(joint_name + "/" + "external_effort");
  }
  return conf;
}

// Fill ordered_interfaces with references to the matching interfaces
// in the same order as in joint_names
template<typename T>
bool get_ordered_interfaces(
  std::vector<T> & unordered_interfaces, const std::vector<std::string> & joint_names,
  const std::string & interface_type, std::vector<std::reference_wrapper<T>> & ordered_interfaces)
{
  for (const auto & joint_name : joint_names) {
    for (auto & command_interface : unordered_interfaces) {
      if (
        (command_interface.get_name() == joint_name) &&
        (command_interface.get_interface_name() == interface_type))
      {
        ordered_interfaces.push_back(std::ref(command_interface));
      }
    }
  }

  return joint_names.size() == ordered_interfaces.size();
}

CallbackReturn CiA402Controller::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::vector<std::reference_wrapper<LoanedCommandInterface>> ordered_interfaces;
  if (
    !get_ordered_interfaces(
      command_interfaces_, joint_names_, hardware_interface::HW_IF_ACCELERATION,
      ordered_interfaces) ||
    command_interfaces_.size() != ordered_interfaces.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu position command interfaces, got %zu",
      joint_names_.size(),
      ordered_interfaces.size());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}
// When deactivating the controller, the effort command on all joints is set to 0
CallbackReturn CiA402Controller::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto index = 0ul; index < joint_names_.size(); ++index) {
    command_interfaces_[index].set_value(0.0);
  }
  return CallbackReturn::SUCCESS;
}
// main control loop function getting the state interface and writing to the command interface
controller_interface::return_type CiA402Controller::update(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  // getting the data from the subscriber using the rt pipe
  auto reference = rt_command_ptr_.readFromRT();

  // no command received yet
  if (!reference || !(*reference)) {
    return controller_interface::return_type::OK;
  }

  // checking reference data validity
  if ((*reference)->joint_names.size() != joint_names_.size() ||
    (*reference)->points[0].positions.size() != joint_names_.size())
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(), 1000, "command size does not match number of interfaces");
    return controller_interface::return_type::ERROR;
  }
  // checkoing if new reference
  if ((*reference)->header.stamp != previous_stamp_) {
    reference_index_ = 0;
    previous_stamp_ = (*reference)->header.stamp;
  }

  for (auto index = 0ul; index < joint_names_.size(); ++index) {
    command_interfaces_[index].set_value(uOpt_(index));
  }

  return controller_interface::return_type::OK;
}

}  // namespace ethercat_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ethercat_controllers::CiA402Controller, controller_interface::ControllerInterface)
