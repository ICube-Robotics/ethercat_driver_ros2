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

#include <iostream>
#include <iomanip>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ethercat_msgs/srv/get_sdo.hpp"
#include "ethercat_msgs/srv/set_sdo.hpp"
#include "ethercat_manager/ec_master_async.hpp"
#include "ethercat_manager/data_convertion_tools.hpp"

namespace ethercat_manager
{
void upload(
  const std::shared_ptr<ethercat_msgs::srv::GetSdo::Request> request,
  std::shared_ptr<ethercat_msgs::srv::GetSdo::Response> response)
{
  ec_ioctl_slave_sdo_upload_t data;
  std::stringstream return_stream, data_stream;
  const DataType * data_type = NULL;
  double data_value = std::numeric_limits<double>::quiet_NaN();
  response->sdo_return_value = data_value;

  data.sdo_index = request->sdo_index;
  data.sdo_entry_subindex = request->sdo_subindex;
  data.slave_position = request->slave_position;

  if (!(data_type = get_data_type(request->sdo_data_type))) {
    return_stream << "Invalid data type '" << request->sdo_data_type << "'!";
    response->sdo_return_message = return_stream.str();
    response->success = false;
    RCLCPP_ERROR(rclcpp::get_logger("ethercat_manager"), return_stream.str().c_str());
    return;
  }

  data.target_size = data_type->byteSize;
  data.target = new uint8_t[data.target_size + 1];

  EcMasterAsync master(request->master_id);
  try {
    master.open(EcMasterAsync::Read);
  } catch (MasterException & e) {
    return_stream << e.what();
    response->success = false;
    RCLCPP_ERROR(rclcpp::get_logger("ethercat_manager"), return_stream.str().c_str());
    response->sdo_return_message = return_stream.str();
    if (nullptr != data.target) {
      delete[] data.target;
    }
    return;
  }

  try {
    master.sdo_upload(&data);
  } catch (MasterException & e) {
    return_stream << e.what();
    response->success = false;
    RCLCPP_ERROR(rclcpp::get_logger("ethercat_manager"), return_stream.str().c_str());
    response->sdo_return_message = return_stream.str();
    if (nullptr != data.target) {
      delete[] data.target;
    }
    return;
  }

  master.close();

  try {
    buffer2data(data_stream, data_value, data_type, data.target, data.data_size);
  } catch (SizeException & e) {
    return_stream << e.what();
    response->success = false;
    RCLCPP_ERROR(rclcpp::get_logger("ethercat_manager"), return_stream.str().c_str());
    response->sdo_return_message = return_stream.str();
    if (nullptr != data.target) {
      delete[] data.target;
    }
    return;
  }
  return_stream << "SDO upload done successfully";
  response->success = true;
  response->sdo_return_value_string = data_stream.str();
  response->sdo_return_value = data_value;
  response->sdo_return_message = return_stream.str();

  if (nullptr != data.target) {
    delete[] data.target;
  }
  RCLCPP_INFO(rclcpp::get_logger("ethercat_sdo_srv_server"), return_stream.str().c_str());
}

void download(
  const std::shared_ptr<ethercat_msgs::srv::SetSdo::Request> request,
  std::shared_ptr<ethercat_msgs::srv::SetSdo::Response> response)
{
  ec_ioctl_slave_sdo_download_t data;
  std::stringstream return_stream, data_stream;
  const DataType * data_type = NULL;
  data.complete_access = 0;
  data.sdo_index = request->sdo_index;
  data.sdo_entry_subindex = request->sdo_subindex;

  data.slave_position = request->slave_position;
  if (!(data_type = get_data_type(request->sdo_data_type))) {
    return_stream << "Invalid data type '" << request->sdo_data_type << "'!";
    response->success = false;
    response->sdo_return_message = return_stream.str();
    RCLCPP_ERROR(rclcpp::get_logger("ethercat_manager"), return_stream.str().c_str());
    return;
  }

  data.data_size = request->sdo_value.size();
  data.data = new uint8_t[data.data_size + 1];

  try {
    data.data_size = data2buffer(
      data_type, request->sdo_value, data.data, data.data_size);
  } catch (SizeException & e) {
    return_stream << e.what();
    response->success = false;
    response->sdo_return_message = return_stream.str();
    RCLCPP_ERROR(rclcpp::get_logger("ethercat_manager"), return_stream.str().c_str());
    if (nullptr != data.data) {
      delete[] data.data;
    }
    return;
  } catch (std::ios::failure & e) {
    return_stream << "Invalid value for type '" << data_type->name << "'!";
    response->success = false;
    response->sdo_return_message = return_stream.str();
    RCLCPP_ERROR(rclcpp::get_logger("ethercat_manager"), return_stream.str().c_str());
    if (nullptr != data.data) {
      delete[] data.data;
    }
    return;
  }

  EcMasterAsync master(request->master_id);
  try {
    master.open(EcMasterAsync::ReadWrite);
  } catch (MasterException & e) {
    return_stream << e.what();
    response->success = false;
    RCLCPP_ERROR(rclcpp::get_logger("ethercat_manager"), return_stream.str().c_str());
    response->sdo_return_message = return_stream.str();
    if (nullptr != data.data) {
      delete[] data.data;
    }
    return;
  }

  try {
    master.sdo_download(&data);
  } catch (MasterException & e) {
    return_stream << e.what();
    response->success = false;
    RCLCPP_ERROR(rclcpp::get_logger("ethercat_manager"), return_stream.str().c_str());
    response->sdo_return_message = return_stream.str();
    if (nullptr != data.data) {
      delete[] data.data;
    }
    return;
  }

  master.close();

  return_stream << "SDO download done successfully";
  response->success = true;
  response->sdo_return_message = return_stream.str();

  if (nullptr != data.data) {
    delete[] data.data;
  }
  RCLCPP_INFO(rclcpp::get_logger("ethercat_sdo_srv_server"), return_stream.str().c_str());
}
}  // namespace ethercat_manager

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("ethercat_sdo_srv_server");

  rclcpp::Service<ethercat_msgs::srv::GetSdo>::SharedPtr service_get_sdo =
    node->create_service<ethercat_msgs::srv::GetSdo>(
    "ethercat_manager/get_sdo",
    &ethercat_manager::upload);

  rclcpp::Service<ethercat_msgs::srv::SetSdo>::SharedPtr service_set_sdo =
    node->create_service<ethercat_msgs::srv::SetSdo>(
    "ethercat_manager/set_sdo",
    &ethercat_manager::download);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
