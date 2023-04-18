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

#ifndef ETHERCAT_MANAGER__EC_MASTER_ASYNC_HPP_
#define ETHERCAT_MANAGER__EC_MASTER_ASYNC_HPP_

#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <ecrt.h>
#include <errno.h>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "ethercat_manager/ec_master_async_io.hpp"


namespace ethercat_manager
{
class EcMasterAsync
{
public:
  explicit EcMasterAsync(uint16_t master_index = 0)
  {
    index_ = master_index;
    mcount_ = 0;
    fd_ = -1;
  }
  ~EcMasterAsync() {close();}

  void close()
  {
    if (fd_ != -1) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  enum Permissions {Read, ReadWrite};

  int open(Permissions perm)
  {
    std::stringstream deviceName;

    if (fd_ == -1) {  // not already open
      ec_ioctl_module_t module_data;
      deviceName << "/dev/EtherCAT" << index_;

      if ((fd_ = ::open(
          deviceName.str().c_str(),
          perm == ReadWrite ? O_RDWR : O_RDONLY)) == -1)
      {
        std::stringstream err;
        err << "Failed to open master device " << deviceName.str() << ": "
            << strerror(errno);
        RCLCPP_ERROR(rclcpp::get_logger("ethercat_manager"), err.str().c_str());
        return -1;
      }

      getModule(&module_data);
      if (module_data.ioctl_version_magic != EC_IOCTL_VERSION_MAGIC) {
        std::stringstream err;
        err << "ioctl() version magic is differing: "
            << deviceName.str() << ": " << module_data.ioctl_version_magic
            << ", ethercat tool: " << EC_IOCTL_VERSION_MAGIC;
        RCLCPP_ERROR(rclcpp::get_logger("ethercat_manager"), err.str().c_str());
        return -1;
      }
      mcount_ = module_data.master_count;
    }
    return 0;
  }

  int sdo_download(ec_ioctl_slave_sdo_download_t * data)
  {
    if (ioctl(fd_, EC_IOCTL_SLAVE_SDO_DOWNLOAD, data) < 0) {
      std::stringstream err;
      if (errno == EIO && data->abort_code) {
        err << "SDO transfer aborted with code " << data->abort_code;
        RCLCPP_ERROR(rclcpp::get_logger("ethercat_manager"), err.str().c_str());
        return -1;
      } else {
        err << "Failed to download SDO: " << strerror(errno);
        RCLCPP_ERROR(rclcpp::get_logger("ethercat_manager"), err.str().c_str());
        return -1;
      }
    }
    return 0;
  }

  int sdo_upload(ec_ioctl_slave_sdo_upload_t * data)
  {
    if (ioctl(fd_, EC_IOCTL_SLAVE_SDO_UPLOAD, data) < 0) {
      std::stringstream err;
      if (errno == EIO && data->abort_code) {
        err << "SDO transfer aborted with code " << data->abort_code;
        RCLCPP_ERROR(rclcpp::get_logger("ethercat_manager"), err.str().c_str());
        return -1;
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("ethercat_manager"), err.str().c_str());
        return -1;
      }
    }
    return 0;
  }

  int getModule(ec_ioctl_module_t * data)
  {
    if (ioctl(fd_, EC_IOCTL_MODULE, data) < 0) {
      std::stringstream err;
      err << "Failed to get module information: " << strerror(errno);
      RCLCPP_ERROR(rclcpp::get_logger("ethercat_manager"), err.str().c_str());
      return -1;
    }
    return 0;
  }

private:
  unsigned int index_;
  unsigned int mcount_;
  int fd_;
};
}  // namespace ethercat_manager
#endif  // ETHERCAT_MANAGER__EC_MASTER_ASYNC_HPP_
