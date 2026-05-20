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

#ifndef ETHERCAT_DRIVER__ETHERCAT_ROS2_CONTROL_XML_PARSER_HPP_
#define ETHERCAT_DRIVER__ETHERCAT_ROS2_CONTROL_XML_PARSER_HPP_

#include <string>
#include <unordered_map>
#include <vector>

namespace ethercat_driver
{

class EthercatRos2ControlXmlParser
{
public:
  static std::vector<std::unordered_map<std::string, std::string>> getEcModuleParam(
    const std::string & urdf,
    const std::string & component_name,
    const std::string & component_type);
};

}  // namespace ethercat_driver

#endif  // ETHERCAT_DRIVER__ETHERCAT_ROS2_CONTROL_XML_PARSER_HPP_
