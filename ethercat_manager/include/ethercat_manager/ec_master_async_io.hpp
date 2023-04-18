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

#ifndef ETHERCAT_MANAGER__EC_MASTER_ASYNC_IO_HPP_
#define ETHERCAT_MANAGER__EC_MASTER_ASYNC_IO_HPP_

#define EC_IOCTL_TYPE 0xa4
#define EC_IO(nr)  _IO(EC_IOCTL_TYPE, nr)
#define EC_IOR(nr, type)  _IOR(EC_IOCTL_TYPE, nr, type)
#define EC_IOW(nr, type)  _IOW(EC_IOCTL_TYPE, nr, type)
#define EC_IOWR(nr, type)  _IOWR(EC_IOCTL_TYPE, nr, type)
#define EC_IOCTL_VERSION_MAGIC 31
#define EC_IOCTL_MODULE  EC_IOR(0x00, ec_ioctl_module_t)
#define EC_IOCTL_SLAVE_SDO_UPLOAD  EC_IOWR(0x0e, ec_ioctl_slave_sdo_upload_t)
#define EC_IOCTL_SLAVE_SDO_DOWNLOAD  EC_IOWR(0x0f, ec_ioctl_slave_sdo_download_t)

typedef struct
{
  uint32_t ioctl_version_magic;
  uint32_t master_count;
} ec_ioctl_module_t;

typedef struct
{
  // inputs
  uint16_t slave_position;
  uint16_t sdo_index;
  uint8_t sdo_entry_subindex;
  size_t target_size;
  uint8_t * target;

  // outputs
  size_t data_size;
  uint32_t abort_code;
} ec_ioctl_slave_sdo_upload_t;

typedef struct
{
  // inputs
  uint16_t slave_position;
  uint16_t sdo_index;
  uint8_t sdo_entry_subindex;
  uint8_t complete_access;
  size_t data_size;
  uint8_t * data;

  // outputs
  uint32_t abort_code;
} ec_ioctl_slave_sdo_download_t;

#endif  // ETHERCAT_MANAGER__EC_MASTER_ASYNC_IO_HPP_
