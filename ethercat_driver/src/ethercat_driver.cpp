// Copyright 2022 ICUBE Laboratory, University of Strasbourg
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

#include <tinyxml2.h>
#include <limits>
#include <regex>
#include <string>
#include <unordered_map>
#include <vector>

#include "ethercat_driver/ethercat_driver.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ethercat_driver
{
CallbackReturn EthercatDriver::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  hw_joint_states_.resize(info_.joints.size());
  for (uint j = 0; j < info_.joints.size(); j++) {
    hw_joint_states_[j].resize(
      info_.joints[j].state_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
  }
  hw_sensor_states_.resize(info_.sensors.size());
  for (uint s = 0; s < info_.sensors.size(); s++) {
    hw_sensor_states_[s].resize(
      info_.sensors[s].state_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
  }
  hw_gpio_states_.resize(info_.gpios.size());
  for (uint g = 0; g < info_.gpios.size(); g++) {
    hw_gpio_states_[g].resize(
      info_.gpios[g].state_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
  }
  hw_joint_commands_.resize(info_.joints.size());
  for (uint j = 0; j < info_.joints.size(); j++) {
    hw_joint_commands_[j].resize(
      info_.joints[j].command_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
  }
  hw_sensor_commands_.resize(info_.sensors.size());
  for (uint s = 0; s < info_.sensors.size(); s++) {
    hw_sensor_commands_[s].resize(
      info_.sensors[s].command_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
  }
  hw_gpio_commands_.resize(info_.gpios.size());
  for (uint g = 0; g < info_.gpios.size(); g++) {
    hw_gpio_commands_[g].resize(
      info_.gpios[g].command_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
  }

  for (uint j = 0; j < info_.joints.size(); j++) {
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "joints");
    // check all joints for EC modules and load into ec_modules_
    if (!info_.joints[j].parameters.at("ec_component").empty()) {
      std::string param = info_.joints[j].parameters.at("ec_component");
      // TODO(mcbed): need find better way to get parameters !
      std::replace(param.begin(), param.end(), '[', '<');
      std::replace(param.begin(), param.end(), ']', '>');
      auto module_params = getEcModuleParam(param);
      ec_module_parameters_.insert(
        ec_module_parameters_.end(), module_params.begin(),
        module_params.end());
      for (auto i = 0ul; i < module_params.size(); i++) {
        for (auto k = 0ul; k < info_.joints[j].state_interfaces.size(); k++) {
          module_params[i]["state_interface/" + info_.joints[j].state_interfaces[k].name] =
            std::to_string(k);
        }
        for (auto k = 0ul; k < info_.joints[j].command_interfaces.size(); k++) {
          module_params[i]["command_interface/" + info_.joints[j].command_interfaces[k].name] =
            std::to_string(k);
        }
        try {
          auto module = ec_loader_.createSharedInstance(module_params[i].at("plugin"));
          if (!module->setupSlave(module_params[i], &hw_joint_states_[j], &hw_joint_commands_[j])) {
            RCLCPP_FATAL(
              rclcpp::get_logger("EthercatDriver"), "Setup of Joint module %i FAILED.",
              i + 1);
            return CallbackReturn::ERROR;
          }
          ec_modules_.push_back(module);
        } catch (pluginlib::PluginlibException & ex) {
          RCLCPP_FATAL(
            rclcpp::get_logger("EthercatDriver"),
            "The plugin of %s failed to load for some reason. Error: %s\n",
            info_.joints[j].name.c_str(), ex.what());
        }
      }
    }
  }
  for (uint g = 0; g < info_.gpios.size(); g++) {
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "gpios");
    // check all gpios for EC modules and load into ec_modules_
    if (!info_.gpios[g].parameters.at("ec_component").empty()) {
      std::string param = info_.gpios[g].parameters.at("ec_component");
      // TODO(mcbed): need find better way to get parameters !
      std::replace(param.begin(), param.end(), '[', '<');
      std::replace(param.begin(), param.end(), ']', '>');
      auto module_params = getEcModuleParam(param);
      ec_module_parameters_.insert(
        ec_module_parameters_.end(), module_params.begin(),
        module_params.end());
      for (auto i = 0ul; i < module_params.size(); i++) {
        for (auto k = 0ul; k < info_.gpios[g].state_interfaces.size(); k++) {
          module_params[i]["state_interface/" + info_.gpios[g].state_interfaces[k].name] =
            std::to_string(k);
        }
        for (auto k = 0ul; k < info_.gpios[g].command_interfaces.size(); k++) {
          module_params[i]["command_interface/" + info_.gpios[g].command_interfaces[k].name] =
            std::to_string(k);
        }
        try {
          auto module = ec_loader_.createSharedInstance(module_params[i].at("plugin"));
          if (!module->setupSlave(module_params[i], &hw_gpio_states_[g], &hw_gpio_commands_[g])) {
            RCLCPP_FATAL(
              rclcpp::get_logger("EthercatDriver"), "Setup of GPIO module %i FAILED.",
              i + 1);
            return CallbackReturn::ERROR;
          }
          ec_modules_.push_back(module);
        } catch (pluginlib::PluginlibException & ex) {
          RCLCPP_FATAL(
            rclcpp::get_logger("EthercatDriver"),
            "The plugin of %s failed to load for some reason. Error: %s\n",
            info_.gpios[g].name.c_str(), ex.what());
        }
      }
    }
  }
  for (uint s = 0; s < info_.sensors.size(); s++) {
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "sensors");
    // check all sensors for EC modules and load into ec_modules_
    if (!info_.sensors[s].parameters.at("ec_component").empty()) {
      std::string param = info_.sensors[s].parameters.at("ec_component");
      // TODO(mcbed): need find better way to get parameters !
      std::replace(param.begin(), param.end(), '[', '<');
      std::replace(param.begin(), param.end(), ']', '>');
      auto module_params = getEcModuleParam(param);
      ec_module_parameters_.insert(
        ec_module_parameters_.end(), module_params.begin(),
        module_params.end());
      for (auto i = 0ul; i < module_params.size(); i++) {
        for (auto k = 0ul; k < info_.sensors[s].state_interfaces.size(); k++) {
          module_params[i]["state_interface/" + info_.sensors[s].state_interfaces[k].name] =
            std::to_string(k);
        }
        for (auto k = 0ul; k < info_.sensors[s].command_interfaces.size(); k++) {
          module_params[i]["command_interface/" + info_.sensors[s].command_interfaces[k].name] =
            std::to_string(k);
        }
        try {
          auto module = ec_loader_.createSharedInstance(module_params[i].at("plugin"));
          if (!module->setupSlave(
              module_params[i], &hw_sensor_states_[s],
              &hw_sensor_commands_[s]))
          {
            RCLCPP_FATAL(
              rclcpp::get_logger("EthercatDriver"), "Setup of Sensor module %i FAILED.",
              i + 1);
            return CallbackReturn::ERROR;
          }
          ec_modules_.push_back(module);
        } catch (pluginlib::PluginlibException & ex) {
          RCLCPP_FATAL(
            rclcpp::get_logger("EthercatDriver"),
            "The plugin of %s failed to load for some reason. Error: %s\n",
            info_.sensors[s].name.c_str(), ex.what());
        }
      }
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Got %i modules", ec_modules_.size());

  return CallbackReturn::SUCCESS;
}

CallbackReturn EthercatDriver::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> EthercatDriver::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  // export joint state interface
  for (uint j = 0; j < info_.joints.size(); j++) {
    for (uint i = 0; i < info_.joints[j].state_interfaces.size(); i++) {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          info_.joints[j].name, info_.joints[j].state_interfaces[i].name, &hw_joint_states_[j][i]));
    }
  }
  // export sensor state interface
  for (uint s = 0; s < info_.sensors.size(); s++) {
    for (uint i = 0; i < info_.sensors[s].state_interfaces.size(); i++) {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          info_.sensors[s].name, info_.sensors[s].state_interfaces[i].name,
          &hw_sensor_states_[s][i]));
    }
  }
  // export gpio state interface
  for (uint g = 0; g < info_.gpios.size(); g++) {
    for (uint i = 0; i < info_.gpios[g].state_interfaces.size(); i++) {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          info_.gpios[g].name, info_.gpios[g].state_interfaces[i].name, &hw_gpio_states_[g][i]));
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> EthercatDriver::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  // export joint command interface
  std::vector<double> test;
  for (uint j = 0; j < info_.joints.size(); j++) {
    for (uint i = 0; i < info_.joints[j].command_interfaces.size(); i++) {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          info_.joints[j].name, info_.joints[j].command_interfaces[i].name,
          &hw_joint_commands_[j][i]));
    }
  }
  // export sensor command interface
  for (uint s = 0; s < info_.sensors.size(); s++) {
    for (uint i = 0; i < info_.sensors[s].command_interfaces.size(); i++) {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          info_.sensors[s].name, info_.sensors[s].command_interfaces[i].name,
          &hw_sensor_commands_[s][i]));
    }
  }
  // export gpio command interface
  for (uint g = 0; g < info_.gpios.size(); g++) {
    for (uint i = 0; i < info_.gpios[g].command_interfaces.size(); i++) {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          info_.gpios[g].name, info_.gpios[g].command_interfaces[i].name,
          &hw_gpio_commands_[g][i]));
    }
  }
  return command_interfaces;
}

CallbackReturn EthercatDriver::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Starting ...please wait...");
  if (info_.hardware_parameters.find("control_frequency") == info_.hardware_parameters.end()) {
    control_frequency_ = 100;
  } else {
    control_frequency_ = std::stod(info_.hardware_parameters["control_frequency"]);
  }

  if (control_frequency_ < 0) {
    RCLCPP_FATAL(rclcpp::get_logger("EthercatDriver"), "Invalid control frequency!");
    return CallbackReturn::ERROR;
  }

  // start EC and wait until state operative

  master_.setCtrlFrequency(control_frequency_);

  for (auto i = 0ul; i < ec_modules_.size(); i++) {
    master_.addSlave(
      std::stod(ec_module_parameters_[i]["alias"]),
      std::stod(ec_module_parameters_[i]["position"]), ec_modules_[i].get());
  }
  master_.activate();
  RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Activated EcMaster!");

  // start after one second
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  t.tv_sec++;

  bool running = true;
  while (running) {
    // wait until next shot
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
    // update EtherCAT bus

    master_.update();
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "updated!");

    // check if operational
    bool isAllInit = true;
    for (auto & module : ec_modules_) {
      isAllInit = isAllInit && module->initialized();
    }
    if (isAllInit) {
      running = false;
    }
    // get actual time
    auto curr_t = std::chrono::system_clock::now();
    // calculate next shot. carry over nanoseconds into microseconds.
    t.tv_nsec += master_.getInterval();
    while (t.tv_nsec >= 1000000000) {
      t.tv_nsec -= 1000000000;
      t.tv_sec++;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "System Successfully started!");

  return CallbackReturn::SUCCESS;
}

CallbackReturn EthercatDriver::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Stopping ...please wait...");

  // stop EC and disconnect
  master_.stop();

  RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "System successfully stopped!");

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type EthercatDriver::read()
{
  master_.readData();
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type EthercatDriver::write()
{
  master_.writeData();
  return hardware_interface::return_type::OK;
}

std::vector<std::unordered_map<std::string, std::string>> EthercatDriver::getEcModuleParam(
  std::string xml)
{
  tinyxml2::XMLDocument doc;
  std::vector<std::unordered_map<std::string, std::string>> module_params;
  std::unordered_map<std::string, std::string> module_param;
  doc.Parse(xml.c_str());
  tinyxml2::XMLElement * pRootElement = doc.RootElement();
  if (NULL != pRootElement) {
    tinyxml2::XMLElement * pEcModule = pRootElement->FirstChildElement("ec_module");
    while (pEcModule) {
      module_param.clear();
      module_param["name"] = pEcModule->Attribute("name");
      tinyxml2::XMLElement * pPlugin = pEcModule->FirstChildElement("plugin");
      if (NULL != pPlugin) {
        module_param["plugin"] = pPlugin->GetText();
      }
      tinyxml2::XMLElement * pParam = pEcModule->FirstChildElement("param");
      while (pParam) {
        module_param[pParam->Attribute("name")] = pParam->GetText();
        pParam = pParam->NextSiblingElement("param");
      }
      module_params.push_back(module_param);
      pEcModule = pEcModule->NextSiblingElement("ec_module");
    }
  }
  return module_params;
}

}  // namespace ethercat_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ethercat_driver::EthercatDriver, hardware_interface::SystemInterface)
