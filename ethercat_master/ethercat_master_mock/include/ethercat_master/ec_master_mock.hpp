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

#ifndef ETHERCAT_MASTER__EC_MASTER_MOCK_HPP_
#define ETHERCAT_MASTER__EC_MASTER_MOCK_HPP_

#include <string>
#include <vector>
#include <map>
#include <memory>

#include "ethercat_interface/ec_master.hpp"
#include "ethercat_interface/ec_slave.hpp"
#include "ethercat_master/ec_slave_mock.hpp"

namespace ethercat_master
{

class MockMaster : public ethercat_interface::EcMaster
{
public:
  MockMaster();
  ~MockMaster();

  bool init(std::string master_interface = "0");

  bool add_slave(ethercat_interface::EcSlave * slave);

  bool configure_slaves();

  bool start();

  void update(uint32_t domain = 0);

  bool spin_slaves_until_operational();

  bool stop();

  void set_ctrl_frequency(double frequency)
  {
    interval_ = 1000000000.0 / frequency;
  }

  uint32_t get_interval() {return interval_;}

  bool read_process_data();
  bool write_process_data();

private:
  uint32_t interval_;
  std::vector<std::shared_ptr<MockSlave>> slave_list_;
};

}  // namespace ethercat_master

#endif  // ETHERCAT_MASTER__EC_MASTER_MOCK_HPP_
