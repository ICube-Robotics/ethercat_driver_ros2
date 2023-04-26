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

  mode_ops_.resize(dof_names_.size(), std::numeric_limits<int>::quiet_NaN());
  control_words_.resize(dof_names_.size(), std::numeric_limits<double>::quiet_NaN());
  reset_faults_.resize(dof_names_.size(), false);

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

  using namespace std::placeholders;
  moo_srv_ptr_ = get_node()->create_service<SwitchMOOSrv>(
    "~/switch_mode_of_operation", std::bind(&CiA402Controller::switch_moo_callback, this, _1, _2));
  reset_fault_srv_ptr_ = get_node()->create_service<ResetFaultSrv>(
    "~/reset_fault", std::bind(&CiA402Controller::reset_fault_callback, this, _1, _2));

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
CiA402Controller::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(dof_names_.size() * 2);
  for (const auto & dof_name : dof_names_) {
    conf.names.push_back(dof_name + "/" + "control_word");
    conf.names.push_back(dof_name + "/" + "mode_of_operation");
    conf.names.push_back(dof_name + "/" + "reset_fault");
  }
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
      msg.modes_of_operation[i] = mode_of_operation_str(state_interfaces_[2 * i].get_value());
      msg.status_words[i] = state_interfaces_[2 * i + 1].get_value();
      msg.drive_states[i] = device_state_str(state_interfaces_[2 * i + 1].get_value());
    }

    rt_drive_state_publisher_->unlockAndPublish();
  }

  // getting the data from services using the rt pipe
  auto moo_request = rt_moo_srv_ptr_.readFromRT();
  auto reset_fault_request = rt_reset_fault_srv_ptr_.readFromRT();

  for (auto i = 0ul; i < dof_names_.size(); i++) {
    if (!moo_request || !(*moo_request)) {
      mode_ops_[i] = state_interfaces_[2 * i].get_value();
    } else {
      if (dof_names_[i] == (*moo_request)->dof_name) {
        mode_ops_[i] = (*moo_request)->mode_of_operation;
      }
    }

    if (reset_fault_request && (*reset_fault_request)) {
      if (dof_names_[i] == (*reset_fault_request)->dof_name) {
        reset_faults_[i] = true;
        rt_reset_fault_srv_ptr_.reset();
      }
    }

    command_interfaces_[2 * i + 1].set_value(mode_ops_[i]);  // mode_of_operation
    command_interfaces_[2 * i + 2].set_value(reset_faults_[i]);  // reset_fault
    reset_faults_[i] = false;
  }

  return controller_interface::return_type::OK;
}

/** returns device state based upon the status_word */
std::string CiA402Controller::device_state_str(uint16_t status_word)
{
  if ((status_word & 0b01001111) == 0b00000000) {
    return "STATE_NOT_READY_TO_SWITCH_ON";
  } else if ((status_word & 0b01001111) == 0b01000000) {
    return "STATE_SWITCH_ON_DISABLED";
  } else if ((status_word & 0b01101111) == 0b00100001) {
    return "STATE_READY_TO_SWITCH_ON";
  } else if ((status_word & 0b01101111) == 0b00100011) {
    return "STATE_SWITCH_ON";
  } else if ((status_word & 0b01101111) == 0b00100111) {
    return "STATE_OPERATION_ENABLED";
  } else if ((status_word & 0b01101111) == 0b00000111) {
    return "STATE_QUICK_STOP_ACTIVE";
  } else if ((status_word & 0b01001111) == 0b00001111) {
    return "STATE_FAULT_REACTION_ACTIVE";
  } else if ((status_word & 0b01001111) == 0b00001000) {
    return "STATE_FAULT";
  }
  return "STATE_UNDEFINED";
}

/** returns mode str based upon the mode_of_operation value */
std::string CiA402Controller::mode_of_operation_str(double mode_of_operation)
{
  if (mode_of_operation == 0) {
    return "MODE_NO_MODE";
  } else if (mode_of_operation == 1) {
    return "MODE_PROFILED_POSITION";
  } else if (mode_of_operation == 3) {
    return "MODE_PROFILED_VELOCITY";
  } else if (mode_of_operation == 4) {
    return "MODE_PROFILED_TORQUE";
  } else if (mode_of_operation == 6) {
    return "MODE_HOMING";
  } else if (mode_of_operation == 7) {
    return "MODE_INTERPOLATED_POSITION";
  } else if (mode_of_operation == 8) {
    return "MODE_CYCLIC_SYNC_POSITION";
  } else if (mode_of_operation == 9) {
    return "MODE_CYCLIC_SYNC_VELOCITY";
  } else if (mode_of_operation == 10) {
    return "MODE_CYCLIC_SYNC_TORQUE";
  }
  return "MODE_UNDEFINED";
}

void CiA402Controller::switch_moo_callback(
  const std::shared_ptr<SwitchMOOSrv::Request> request,
  std::shared_ptr<SwitchMOOSrv::Response> response
)
{
  if (find(dof_names_.begin(), dof_names_.end(), request->dof_name) != dof_names_.end()) {
    rt_moo_srv_ptr_.writeFromNonRT(request);
    response->return_message = "Request transmitted to drive at dof:" + request->dof_name;
  } else {
    response->return_message = "Abort. DoF " + request->dof_name + " not configured.";
  }
}

void CiA402Controller::reset_fault_callback(
  const std::shared_ptr<ResetFaultSrv::Request> request,
  std::shared_ptr<ResetFaultSrv::Response> response
)
{
  if (find(dof_names_.begin(), dof_names_.end(), request->dof_name) != dof_names_.end()) {
    rt_reset_fault_srv_ptr_.writeFromNonRT(request);
    response->return_message = "Request transmitted to drive at dof:" + request->dof_name;
  } else {
    response->return_message = "Abort. DoF " + request->dof_name + " not configured.";
  }
}

}  // namespace ethercat_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ethercat_controllers::CiA402Controller, controller_interface::ControllerInterface)
