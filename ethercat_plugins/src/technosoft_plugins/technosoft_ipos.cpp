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

#include "ethercat_plugins/technosoft_plugins/technosoft_ipos.hpp"

namespace ethercat_plugins
{

class Technosoft_IPOS_3604 : public Technosoft_IPOS
{
public:
  Technosoft_IPOS_3604()
  : Technosoft_IPOS(0x000001a3, 0x01ab46e5) {}
  virtual ~Technosoft_IPOS_3604() {}
};

class Technosoft_IPOS_4808BX : public Technosoft_IPOS
{
public:
  Technosoft_IPOS_4808BX()
  : Technosoft_IPOS(0x000001a3, 0x019f418d) {}
  virtual ~Technosoft_IPOS_4808BX() {}
};

class Technosoft_IPOS_4808SY : public Technosoft_IPOS
{
public:
  Technosoft_IPOS_4808SY()
  : Technosoft_IPOS(0x000001a3, 0x019fd2e0) {}
  virtual ~Technosoft_IPOS_4808SY() {}
};

}  // namespace ethercat_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ethercat_plugins::Technosoft_IPOS_3604, ethercat_interface::EcSlave)
PLUGINLIB_EXPORT_CLASS(ethercat_plugins::Technosoft_IPOS_4808BX, ethercat_interface::EcSlave)
PLUGINLIB_EXPORT_CLASS(ethercat_plugins::Technosoft_IPOS_4808SY, ethercat_interface::EcSlave)
