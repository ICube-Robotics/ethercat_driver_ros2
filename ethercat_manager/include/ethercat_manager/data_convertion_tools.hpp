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

#ifndef ETHERCAT_MANAGER__DATA_CONVERTION_TOOLS_HPP_
#define ETHERCAT_MANAGER__DATA_CONVERTION_TOOLS_HPP_

#include <stdint.h>
#include <string>
#include <stdexcept>
#include <ostream>
#include <iostream>
#include "rclcpp/rclcpp.hpp"

namespace ethercat_manager
{

class SizeException
  : public std::runtime_error
{
public:
  explicit SizeException(const std::string & msg)
  : std::runtime_error(msg) {}
};

struct DataType
{
  const char * name;
  uint16_t code;
  size_t byteSize;
};

static const DataType dataTypes[] = {
  {"bool", 0x0001, 1},
  {"int8", 0x0002, 1},
  {"int16", 0x0003, 2},
  {"int32", 0x0004, 4},
  {"uint8", 0x0005, 1},
  {"uint16", 0x0006, 2},
  {"uint32", 0x0007, 4},
  {"float", 0x0008, 4},
  {"string", 0x0009, 0},           // a. k. a. visible_string
  {"octet_string", 0x000a, 0},
  {"unicode_string", 0x000b, 0},
  // ... not implemented yet
  {"int24", 0x0010, 3},
  {"double", 0x0011, 8},
  {"int40", 0x0012, 5},
  {"int48", 0x0013, 6},
  {"int56", 0x0014, 7},
  {"int64", 0x0015, 8},
  {"uint24", 0x0016, 3},
  // reserved        0x0017
  {"uint40", 0x0018, 5},
  {"uint48", 0x0019, 6},
  {"uint56", 0x001a, 7},
  {"uint64", 0x001b, 8},
  // reserved        0x001c-0x001f
  {"sm8", 0xfffb, 1},              // sign-and-magnitude coding
  {"sm16", 0xfffc, 2},             // sign-and-magnitude coding
  {"sm32", 0xfffd, 4},             // sign-and-magnitude coding
  {"sm64", 0xfffe, 8},             // sign-and-magnitude coding
  {"raw", 0xffff, 0},
  {}
};

static const DataType * get_data_type(const std::string & str)
{
  const DataType * d;
  for (d = dataTypes; d->name; d++) {
    if (str == d->name) {
      return d;
    }
  }
  return NULL;
}

static const DataType * get_data_type(uint16_t code)
{
  const DataType * d;
  for (d = dataTypes; d->name; d++) {
    if (code == d->code) {
      return d;
    }
  }
  return NULL;
}

static void buffer2raw(std::ostream & o, const uint8_t * data, size_t size)
{
  o << std::hex << std::setfill('0');
  while (size--) {
    o << " 0x" << std::setw(2) << (unsigned int) *data++;
  }
}

static size_t data2buffer(
  const DataType * type, const std::string & source, void * target,
  size_t targetSize)
{
  std::stringstream str;
  size_t dataSize = type->byteSize;

  memset(target, 0, targetSize);

  str << source;
  str >> resetiosflags(std::ios::basefield);    // guess base from prefix
  str.exceptions(std::ios::failbit);

  switch (type->code) {
    case 0x0001:     // bool
      {
        int16_t val;         // uint8_t is interpreted as char
        str >> val;
        if (val > 1 || val < 0) {
          throw  std::ios::failure("Value out of range");
        }
        *reinterpret_cast<uint8_t *>(target) = val;
        break;
      }
    case 0x0002:     // int8
      {
        int16_t val;         // uint8_t is interpreted as char
        str >> val;
        if (val > 127 || val < -128) {
          throw  std::ios::failure("Value out of range");
        }
        *reinterpret_cast<uint8_t *>(target) = val;
        break;
      }
    case 0x0003:     // int16
      {
        int16_t val;
        str >> val;
        *reinterpret_cast<int16_t *>(target) = cpu_to_le16(val);
        break;
      }
    case 0x0004:     // int32
      {
        int32_t val;
        str >> val;
        *reinterpret_cast<int32_t *>(target) = cpu_to_le32(val);
        break;
      }
    case 0x0005:     // uint8
      {
        uint16_t val;         // uint8_t is interpreted as char
        str >> val;
        if (val > 0xff) {
          throw  std::ios::failure("Value out of range");
        }
        *reinterpret_cast<uint8_t *>(target) = val;
        break;
      }
    case 0x0006:     // uint16
      {
        uint16_t val;
        str >> val;
        *reinterpret_cast<uint16_t *>(target) = cpu_to_le16(val);
        break;
      }
    case 0x0007:     // uint32
      {
        uint32_t val;
        str >> val;
        *reinterpret_cast<uint32_t *>(target) = cpu_to_le32(val);
        break;
      }
    case 0x0008:     // float
      {
        float val;
        str >> val;
        *reinterpret_cast<uint32_t *>(target) =
          cpu_to_le32(*reinterpret_cast<uint32_t *>(reinterpret_cast<void *>( &val)));
        break;
      }
    case 0x0009:     // std::string
    case 0x000a:     // octet_string
    case 0x000b:     // unicode_string
      dataSize = str.str().size();
      if (dataSize > targetSize) {
        std::stringstream err;
        err << "String too large ("
            << dataSize << " > " << targetSize << ")";
        throw SizeException(err.str());
      }
      str.read(reinterpret_cast<char *>(target), dataSize);
      break;
    case 0x0011:     // double
      {
        double val;
        str >> val;
        *reinterpret_cast<uint64_t *>(target) =
          cpu_to_le64(*reinterpret_cast<uint64_t *>(reinterpret_cast<void *>(&val)));
        break;
      }
      break;
    case 0x0015:     // int64
      {
        int64_t val;
        str >> val;
        *reinterpret_cast<int64_t *>(target) = cpu_to_le64(val);
        break;
      }
      break;
    case 0x001b:     // uint64
      {
        uint64_t val;
        str >> val;
        *reinterpret_cast<uint64_t *>(target) = cpu_to_le64(val);
        break;
      }
      break;

    case 0x0010:     // int24
    case 0x0012:     // int40
    case 0x0013:     // int48
    case 0x0014:     // int56
    case 0x0016:     // uint24
    case 0x0018:     // uint40
    case 0x0019:     // uint48
    case 0x001a:     // uint56
      {
        std::stringstream err;
        err << "Non-native integer type " << type->name
            << " is not yet implemented.";
        throw std::runtime_error(err.str());
      }

    case 0xfffb:     // sm8
    case 0xfffc:     // sm16
    case 0xfffd:     // sm32
    case 0xfffe:     // sm64
      {
        std::stringstream err;
        err << "Sign-and-magitude types not yet"
          " implemented for input direction.";
        throw std::runtime_error(err.str());
      }

    default:
      {
        std::stringstream err;
        err << "Unknown data type 0x" << std::hex << type->code;
        throw std::runtime_error(err.str());
      }
  }

  return dataSize;
}

static void buffer2data(
  std::ostream & o, double & value,
  const DataType * type, void * data, size_t dataSize)
{
  uint16_t typeCode;

  if (type) {
    if (type->byteSize && dataSize != type->byteSize) {
      std::stringstream err;
      err << "Data type mismatch. Expected " << type->name
          << " with " << type->byteSize << " byte, but got "
          << dataSize << " byte.";
      throw SizeException(err.str());
    }
    typeCode = type->code;
  } else {
    typeCode = 0xffff;     // raw data
  }

  o << std::setfill('0');

  switch (typeCode) {
    case 0x0001:     // bool
      {
        int val = static_cast<int>(*reinterpret_cast<int8_t *>(data));
        o << "0x" << std::hex << std::setw(2) << val;
        value = static_cast<double>(val);
      }
      break;
    case 0x0002:     // int8
      {
        int val = static_cast<int>(*reinterpret_cast<int8_t *>(data));
        o << "0x" << std::hex << std::setw(2) << val;
        value = static_cast<double>(val);
      }
      break;
    case 0x0003:     // int16
      {
        int16_t val = le16_to_cpup(data);
        o << "0x" << std::hex << std::setw(4) << val;
        value = static_cast<double>(val);
      }
      break;
    case 0x0004:     // int32
      {
        int32_t val = le32_to_cpup(data);
        o << "0x" << std::hex << std::setw(8) << val;
        value = static_cast<double>(val);
      }
      break;
    case 0x0005:     // uint8
      {
        unsigned int val = static_cast<unsigned int>(*reinterpret_cast<uint8_t *>(data));
        o << "0x" << std::hex << std::setw(2) << val;
        value = static_cast<double>(val);
      }
      break;
    case 0x0006:     // uint16
      {
        uint16_t val = le16_to_cpup(data);
        o << "0x" << std::hex << std::setw(4) << val;
        value = static_cast<double>(val);
      }
      break;
    case 0x0007:     // uint32
      {
        uint32_t val = le32_to_cpup(data);
        o << "0x" << std::hex << std::setw(8) << val;
        value = static_cast<double>(val);
      }
      break;
    case 0x0008:     // float
      {
        uint32_t val = le32_to_cpup(data);
        float fval = *reinterpret_cast<float *>(reinterpret_cast<void *>(&val));
        o << fval;
        value = static_cast<double>(val);
      }
      break;
    case 0x0009:     // string
      o << std::string(reinterpret_cast<const char *>(data), dataSize);
      break;
    case 0x000a:     // octet_string
      o << std::string(reinterpret_cast<const char *>(data), dataSize) << std::flush;
      break;
    case 0x000b:     // unicode_string
                     // FIXME encoding
      o << std::string(reinterpret_cast<const char *>(data), dataSize);
      break;
    case 0x0011:     // double
      {
        uint64_t val = le64_to_cpup(data);
        double fval = *reinterpret_cast<double *>(reinterpret_cast<void *>(&val));
        o << fval;
        value = static_cast<double>(val);
      }
      break;
    case 0x0015:     // int64
      {
        int64_t val = le64_to_cpup(data);
        o << "0x" << std::hex << std::setw(16) << val;
        value = static_cast<double>(val);
      }
      break;
    case 0x001b:     // uint64
      {
        uint64_t val = le64_to_cpup(data);
        o << "0x" << std::hex << std::setw(16) << val;
        value = static_cast<double>(val);
      }
      break;
    case 0xfffb:     // sm8
      {
        int8_t val = *reinterpret_cast<uint8_t *>(data);
        int8_t smval = val < 0 ? (val & 0x7f) * -1 : val;

        o << "0x" << std::hex << std::setw(2) << static_cast<int>(val);
        value = static_cast<double>(static_cast<int>(smval));
      }
      break;
    case 0xfffc:     // sm16
      {
        int16_t val = le16_to_cpup(data);
        int16_t smval = val < 0 ? (val & 0x7fff) * -1 : val;

        o << "0x" << std::hex << std::setw(4) << val;
        value = static_cast<double>(smval);
      }
      break;
    case 0xfffd:     // sm32
      {
        int32_t val = le32_to_cpup(data);
        int32_t smval = val < 0 ? (val & 0x7fffffffUL) * -1 : val;

        o << "0x" << std::hex << std::setw(8) << val;
        value = static_cast<double>(smval);
      }
      break;
    case 0xfffe:     // sm64
      {
        int64_t val = le64_to_cpup(data);
        int64_t smval =
          val < 0 ? (val & 0x7fffffffffffffffULL) * -1 : val;

        o << "0x" << std::hex << std::setw(16) << val;
        value = static_cast<double>(smval);
      }
      break;

    default:
      buffer2raw(o, (const uint8_t *) data, dataSize);       // FIXME
      break;
  }
}

}  // namespace ethercat_manager

#endif  // ETHERCAT_MANAGER__DATA_CONVERTION_TOOLS_HPP_
