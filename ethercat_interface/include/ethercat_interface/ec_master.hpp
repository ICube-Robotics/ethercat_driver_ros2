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

namespace ethercat_interface
{
class EcMaster
{
public:
  EcMaster();
  virtual ~EcMaster();

  /** \brief add a slave device to the master */
  virtual void add_slave(EcSlave * slave);

  /** \brief configure slave using SDO */
  virtual int config_slave_sdo(
    uint16_t slave_position, SdoConfigEntry sdo_config,
    uint32_t * abort_code);

  virtual bool init(std::string iface);

  virtual void activate();

  virtual void deactivate();

  virtual bool spin_slave_until_operational();

  virtual bool read_process_data();

  virtual void write_process_data();

  void set_ctrl_frequency(double frequency)
  {
    interval_ = 1000000000.0 / frequency;
  }

protected:
  uint32_t interval_;
};
}  // namespace ethercat_interface
#endif  // ETHERCAT_INTERFACE__EC_MASTER_HPP_
