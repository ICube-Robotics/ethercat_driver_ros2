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

#include <gtest/gtest.h>
#include <memory>

#include <pluginlib/class_loader.hpp>
#include "ethercat_interface/ec_slave.hpp"

TEST(TestLoadAdvantech_AMAX5051, load_ec_module)
{
  pluginlib::ClassLoader<ethercat_interface::EcSlave> ec_loader_{"ethercat_interface", "ethercat_interface::EcSlave"};
  ASSERT_NO_THROW(ec_loader_.createSharedInstance("ethercat_plugins/Advantech_AMAX5051"));
}

TEST(TestLoadAdvantech_AMAX5056, load_ec_module)
{
  pluginlib::ClassLoader<ethercat_interface::EcSlave> ec_loader_{"ethercat_interface", "ethercat_interface::EcSlave"};
  ASSERT_NO_THROW(ec_loader_.createSharedInstance("ethercat_plugins/Advantech_AMAX5056"));
}
