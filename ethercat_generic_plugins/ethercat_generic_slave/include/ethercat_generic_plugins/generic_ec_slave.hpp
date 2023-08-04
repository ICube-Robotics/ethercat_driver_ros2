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
//
// Author: Maciej Bednarczyk (macbednarczyk@gmail.com)

#ifndef ETHERCAT_GENERIC_PLUGINS__GENERIC_EC_SLAVE_HPP_
#define ETHERCAT_GENERIC_PLUGINS__GENERIC_EC_SLAVE_HPP_

#include <vector>
#include <string>
#include <unordered_map>

#include "yaml-cpp/yaml.h"
#include "ethercat_interface/ec_slave.hpp"

namespace ethercat_generic_plugins
{

class GenericEcSlave : public ethercat_interface::EcSlave
{
public:
  GenericEcSlave();
  ~GenericEcSlave();

  int process_data(size_t index, uint8_t * domain_address);

  bool setup_slave(
    std::unordered_map<std::string, std::string> slave_paramters,
    std::vector<double> * state_interface,
    std::vector<double> * command_interface);

  YAML::Node get_slave_config();

protected:
  YAML::Node slave_config_;

  /** set up of the drive configuration from yaml node*/
  bool setup_from_config(YAML::Node slave_config);
  /** set up of the drive configuration from yaml file*/
  bool setup_from_config_file(std::string config_file);

  void setup_interface_mapping();
};
}  // namespace ethercat_generic_plugins

#endif  // ETHERCAT_GENERIC_PLUGINS__GENERIC_EC_SLAVE_HPP_
