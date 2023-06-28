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

#include "ethercat_interface/ec_slave.hpp"

namespace ethercat_plugins
{

class Beckhoff_EK1100 : public ethercat_interface::EcSlave
{
public:
  Beckhoff_EK1100()
  : EcSlave(0x00000002, 0x044c2c52)
  {
    std::cerr << "The Beckhoff_EK1100 plugin is depreciated and will be removed in the future."
              << "Use the GenericEcSlave plugin instead." << std::endl;
  }
  virtual ~Beckhoff_EK1100() {}
};

}  // namespace ethercat_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ethercat_plugins::Beckhoff_EK1100, ethercat_interface::EcSlave)
