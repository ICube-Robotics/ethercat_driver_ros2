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

#ifndef ETHERCAT_INTERFACE__EC_SYNC_MANAGER_HPP_
#define ETHERCAT_INTERFACE__EC_SYNC_MANAGER_HPP_

#include <ecrt.h>
#include <string>
#include <vector>
#include <limits>

#include "yaml-cpp/yaml.h"

namespace ethercat_interface
{

class SMConfig
{
public:
  SMConfig() {}
  ~SMConfig() {}

  bool load_from_config(YAML::Node sm_config)
  {
    // index
    if (sm_config["index"]) {
      index = sm_config["index"].as<uint8_t>();
    } else {
      std::cerr << "missing sdo index info" << std::endl;
      return false;
    }
    // type
    if (sm_config["type"]) {
      if (sm_config["type"].as<std::string>() == "input") {
        type = EC_DIR_INPUT;
      } else if (sm_config["type"].as<std::string>() == "output") {
        type = EC_DIR_OUTPUT;
      } else {
        std::cerr << "sm " << index << ": type should be input/output" << std::endl;
        return false;
      }
    } else {
      std::cerr << "sm " << index << ": missing type info" << std::endl;
      return false;
    }
    // pdo name
    if (sm_config["pdo"]) {
      if (sm_config["pdo"].as<std::string>() == "rpdo") {
        pdo_name = "rpdo";
      } else if (sm_config["pdo"].as<std::string>() == "tpdo") {
        pdo_name = "tpdo";
      }
    }
    // watchdog
    if (sm_config["watchdog"]) {
      if (sm_config["watchdog"].as<std::string>() == "enable") {
        watchdog = EC_WD_ENABLE;
      } else if (sm_config["watchdog"].as<std::string>() == "disable") {
        watchdog = EC_WD_DISABLE;
      }
    }

    return true;
  }

  uint8_t index;
  ec_direction_t type;
  std::string pdo_name = "null";
  ec_watchdog_mode_t watchdog = EC_WD_DEFAULT;
};

}  // namespace ethercat_interface
#endif  // ETHERCAT_INTERFACE__EC_SYNC_MANAGER_HPP_
