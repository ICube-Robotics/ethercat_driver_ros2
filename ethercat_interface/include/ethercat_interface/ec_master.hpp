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
// Author: Maciej Bednarczyk (mcbed.robotics@gmail.com)

#ifndef ETHERCAT_INTERFACE__EC_MASTER_HPP_
#define ETHERCAT_INTERFACE__EC_MASTER_HPP_

#include <string>
#include <memory>
#include "ethercat_interface/ec_slave.hpp"

namespace ethercat_interface
{
class EcMaster
{
public:
  EcMaster() {}
  virtual ~EcMaster() {}

  /** \brief add a slave device to the master */
  virtual bool add_slave(std::shared_ptr<EcSlave> slave) = 0;

  /** \brief configure slave using SDO */
  virtual bool configure_slaves() = 0;

  virtual bool init(std::string iface) = 0;

  virtual bool start() = 0;

  virtual bool stop() = 0;

  virtual bool spin_slaves_until_operational() = 0;

  virtual bool read_process_data() = 0;

  virtual bool write_process_data() = 0;

  void set_ctrl_frequency(double frequency)
  {
    interval_ = 1000000000.0 / frequency;
  }

protected:
  uint32_t interval_;
};
}  // namespace ethercat_interface
#endif  // ETHERCAT_INTERFACE__EC_MASTER_HPP_
