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
#include <map>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "ethercat_manager/ec_master_async_io.hpp"


namespace ethercat_manager
{

class MasterException
  : public std::runtime_error
{
public:
  explicit MasterException(const std::string & msg)
  : std::runtime_error(msg) {}
};

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

  void open(Permissions perm)
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
        throw MasterException(err.str());
      }

      getModule(&module_data);
      if (module_data.ioctl_version_magic != EC_IOCTL_VERSION_MAGIC) {
        std::stringstream err;
        err << "ioctl() version magic is differing: "
            << deviceName.str() << ": " << module_data.ioctl_version_magic
            << ", ethercat tool: " << EC_IOCTL_VERSION_MAGIC;
        throw MasterException(err.str());
      }
      mcount_ = module_data.master_count;
    }
  }

  void sdo_download(ec_ioctl_slave_sdo_download_t * data)
  {
    if (ioctl(fd_, EC_IOCTL_SLAVE_SDO_DOWNLOAD, data) < 0) {
      std::stringstream err;
      if (errno == EIO && data->abort_code) {
        err << "SDO transfer aborted: " << abort_code_map_.find(data->abort_code)->second;
        throw MasterException(err.str());
      } else {
        err << "Failed to download SDO: " << strerror(errno);
        throw MasterException(err.str());
      }
    }
  }

  void sdo_upload(ec_ioctl_slave_sdo_upload_t * data)
  {
    if (ioctl(fd_, EC_IOCTL_SLAVE_SDO_UPLOAD, data) < 0) {
      std::stringstream err;
      if (errno == EIO && data->abort_code) {
        err << "SDO transfer aborted: " << abort_code_map_.find(data->abort_code)->second;
        throw MasterException(err.str());
      } else {
        err << "Failed to upload SDO: " << strerror(errno);
        throw MasterException(err.str());
      }
    }
  }

  void getModule(ec_ioctl_module_t * data)
  {
    if (ioctl(fd_, EC_IOCTL_MODULE, data) < 0) {
      std::stringstream err;
      err << "Failed to get module information: " << strerror(errno);
      throw MasterException(err.str());
    }
  }

private:
  unsigned int index_;
  unsigned int mcount_;
  int fd_;

  const std::map<uint32_t, std::string> abort_code_map_ = {
    {0x05030000, "Toggle bit not changed"},
    {0x05040000, "SDO protocol timeout"},
    {0x05040001, "Client/Server command specifier not valid or unknown"},
    {0x05040005, "Out of memory"},
    {0x06010000, "Unsupported access to an object"},
    {0x06010001, "Attempt to read a write-only object"},
    {0x06010002, "Attempt to write a read-only object"},
    {0x06020000, "This object does not exist in the object directory"},
    {0x06040041, "The object cannot be mapped into the PDO"},
    {0x06040042, "The number and length of the objects to be mapped would"
      " exceed the PDO length"},
    {0x06040043, "General parameter incompatibility reason"},
    {0x06040047, "General internal incompatibility in device"},
    {0x06060000, "Access failure due to a hardware error"},
    {0x06070010, "Data type does not match, length of service parameter does"
      " not match"},
    {0x06070012, "Data type does not match, length of service parameter too"
      " high"},
    {0x06070013, "Data type does not match, length of service parameter too"
      " low"},
    {0x06090011, "Subindex does not exist"},
    {0x06090030, "Value range of parameter exceeded"},
    {0x06090031, "Value of parameter written too high"},
    {0x06090032, "Value of parameter written too low"},
    {0x06090036, "Maximum value is less than minimum value"},
    {0x08000000, "General error"},
    {0x08000020, "Data cannot be transferred or stored to the application"},
    {0x08000021, "Data cannot be transferred or stored to the application"
      " because of local control"},
    {0x08000022, "Data cannot be transferred or stored to the application"
      " because of the present device state"},
    {0x08000023, "Object dictionary dynamic generation fails or no object"
      " dictionary is present"},
    {}
  };
};
}  // namespace ethercat_manager
#endif  // ETHERCAT_MANAGER__EC_MASTER_ASYNC_HPP_
