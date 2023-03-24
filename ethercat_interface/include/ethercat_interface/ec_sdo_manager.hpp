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

#ifndef ETHERCAT_INTERFACE__EC_SDO_MANAGER_HPP_
#define ETHERCAT_INTERFACE__EC_SDO_MANAGER_HPP_

#include <ecrt.h>
#include <string>
#include <vector>
#include <limits>

#include "yaml-cpp/yaml.h"

namespace ethercat_interface
{

class SdoConfigEntry
{
public:
  SdoConfigEntry() {}
  ~SdoConfigEntry() {}

  void buffer_write(uint8_t * buffer)
  {
    if (data_type == "uint8") {
      EC_WRITE_U8(buffer, static_cast<uint8_t>(data));
    } else if (data_type == "int8") {
      EC_WRITE_S8(buffer, static_cast<int8_t>(data));
    } else if (data_type == "uint16") {
      EC_WRITE_U16(buffer, static_cast<uint16_t>(data));
    } else if (data_type == "int16") {
      EC_WRITE_S16(buffer, static_cast<int16_t>(data));
    } else if (data_type == "uint32") {
      EC_WRITE_U32(buffer, static_cast<uint32_t>(data));
    } else if (data_type == "int32") {
      EC_WRITE_S32(buffer, static_cast<int32_t>(data));
    } else if (data_type == "uint64") {
      EC_WRITE_U64(buffer, static_cast<uint64_t>(data));
    } else if (data_type == "int64") {
      EC_WRITE_S64(buffer, static_cast<int64_t>(data));
    }
  }

  bool load_from_config(YAML::Node sdo_config)
  {
    // index
    if (sdo_config["index"]) {
      index = sdo_config["index"].as<uint16_t>();
    } else {
      std::cerr << "missing sdo index info" << std::endl;
      return false;
    }
    // sub_index
    if (sdo_config["sub_index"]) {
      sub_index = sdo_config["sub_index"].as<uint8_t>();
    } else {
      std::cerr << "sdo " << index << ": missing sdo info" << std::endl;
      return false;
    }
    // data type
    if (sdo_config["type"]) {
      data_type = sdo_config["type"].as<std::string>();
    } else {
      std::cerr << "sdo " << index << ": missing sdo data type info" << std::endl;
      return false;
    }
    // value
    if (sdo_config["value"]) {
      data = sdo_config["value"].as<int>();
    } else {
      std::cerr << "sdo " << index << ": missing sdo value" << std::endl;
      return false;
    }

    return true;
  }

  size_t data_size()
  {
    return type2bytes(data_type);
  }

  uint16_t index;
  uint8_t sub_index;
  std::string data_type;
  int data;

private:
  size_t type2bytes(std::string type)
  {
    if (type == "int8" || type == "uint8") {
      return 1;
    } else if (type == "int16" || type == "uint16") {
      return 2;
    } else if (type == "int32" || type == "uint32") {
      return 4;
    } else if (type == "int64" || type == "uint64") {
      return 8;
    }
  }
};

}  // namespace ethercat_interface
#endif  // ETHERCAT_INTERFACE__EC_SDO_MANAGER_HPP_
