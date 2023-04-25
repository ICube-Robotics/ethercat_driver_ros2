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

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "ethercat_controllers/generic_cia402_controller.hpp"

namespace ethercat_controllers
{
using hardware_interface::LoanedCommandInterface;

CiA402Controller::CiA402Controller()
: controller_interface::ControllerInterface(),
  rt_drive_state_publisher_(nullptr)
{
}

CallbackReturn CiA402Controller::on_init()
{
  try {
    // definition of the parameters that need to be queried from the
    // controller configuration file with default values
    auto_declare<std::vector<std::string>>("dofs", std::vector<std::string>());
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn CiA402Controller::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // getting the names of the dofs to be controlled
  dof_names_ = get_node()->get_parameter("dofs").as_string_array();

  if (dof_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'dofs' parameter was empty");
    return CallbackReturn::FAILURE;
  }

  try {
    // register data publisher
    drive_state_publisher_ = get_node()->create_publisher<DriveStateMsgType>(
      "~/drive_states", rclcpp::SystemDefaultsQoS());
    rt_drive_state_publisher_ = std::make_unique<DriveStatePublisher>(drive_state_publisher_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Exception thrown during publisher creation at configure stage"
      "with message : %s \n",
      e.what()
    );
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
CiA402Controller::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::NONE;
  return conf;
}

controller_interface::InterfaceConfiguration
CiA402Controller::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(dof_names_.size() * 2);
  for (const auto & dof_name : dof_names_) {
    conf.names.push_back(dof_name + "/" + "mode_of_operation");
    conf.names.push_back(dof_name + "/" + "status_word");
  }
  return conf;
}

CallbackReturn CiA402Controller::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn CiA402Controller::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type CiA402Controller::update(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  if (rt_drive_state_publisher_ && rt_drive_state_publisher_->trylock()) {
    auto & msg = rt_drive_state_publisher_->msg_;
    msg.header.stamp = get_node()->now();
    msg.dof_names.resize(dof_names_.size());
    msg.modes_of_operation.resize(dof_names_.size());
    msg.status_words.resize(dof_names_.size());
    msg.drive_states.resize(dof_names_.size());

    for (auto i = 0ul; i < dof_names_.size(); i++) {
      msg.dof_names[i] = dof_names_[i];
      msg.modes_of_operation[i] = state_interfaces_[2 * i].get_value();
      msg.status_words[i] = state_interfaces_[2 * i + 1].get_value();
      msg.drive_states[i] = deviceState(state_interfaces_[2 * i + 1].get_value());
    }

    rt_drive_state_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

/** returns device state based upon the status_word */
int CiA402Controller::deviceState(uint16_t status_word)
{
  if ((status_word & 0b01001111) == 0b00000000) {
    return 2;
  } else if ((status_word & 0b01001111) == 0b01000000) {
    return 3;
  } else if ((status_word & 0b01101111) == 0b00100001) {
    return 4;
  } else if ((status_word & 0b01101111) == 0b00100011) {
    return 5;
  } else if ((status_word & 0b01101111) == 0b00100111) {
    return 6;
  } else if ((status_word & 0b01101111) == 0b00000111) {
    return 7;
  } else if ((status_word & 0b01001111) == 0b00001111) {
    return 8;
  } else if ((status_word & 0b01001111) == 0b00001000) {
    return 9;
  }
  return 0;
}

}  // namespace ethercat_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ethercat_controllers::CiA402Controller, controller_interface::ControllerInterface)
