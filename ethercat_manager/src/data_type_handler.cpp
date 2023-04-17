/*****************************************************************************
 *
 *  Copyright (C) 2006-2022  Florian Pose, Ingenieurgemeinschaft IgH
 *
 *  This file is part of the IgH EtherCAT Master.
 *
 *  The IgH EtherCAT Master is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License version 2, as
 *  published by the Free Software Foundation.
 *
 *  The IgH EtherCAT Master is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 *  Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with the IgH EtherCAT Master; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 *  ---
 *
 *  The license mentioned above concerns the source code only. Using the
 *  EtherCAT technology and brand is only permitted in compliance with the
 *  industrial property and similar rights of Beckhoff Automation GmbH.
 *
 ****************************************************************************/

#define DEBUG 0

#if DEBUG
#include <iostream>
#endif

#include <string.h>
#include <iomanip>
#include <sstream>
using namespace std;

#include "ethercat_manager/data_type_handler.h"

#include "ecrt.h"

/*****************************************************************************/

DataTypeHandler::DataTypeHandler()
{
}

/****************************************************************************/

const DataTypeHandler::DataType * DataTypeHandler::findDataType(
  const string & str
)
{
  const DataType * d;

  for (d = dataTypes; d->name; d++) {
    if (str == d->name) {
      return d;
    }
  }

  return NULL;   // FIXME exception
}

/****************************************************************************/

string DataTypeHandler::typeInfo()
{
  stringstream s;

  s
    << "These are valid data types to use with" << endl
    << "the --type option:" << endl
    << "  bool," << endl
    << "  int8, int16, int32, int64," << endl
    << "  uint8, uint16, uint32, uint64," << endl
    << "  float, double," << endl
    << "  string, octet_string, unicode_string." << endl
    << "For sign-and-magnitude coding, use the following types:" << endl
    << "  sm8, sm16, sm32, sm64" << endl;
  return s.str();
}

/****************************************************************************/

const DataTypeHandler::DataType * DataTypeHandler::findDataType(uint16_t code)
{
  const DataType * d;

  for (d = dataTypes; d->name; d++) {
    if (code == d->code) {
      return d;
    }
  }

  return NULL;
}

/****************************************************************************/

size_t DataTypeHandler::interpretAsType(
  const DataType * type,
  const string & source,
  void * target,
  size_t targetSize
)
{
  stringstream str;
  size_t dataSize = type->byteSize;

  memset(target, 0, targetSize);

#if DEBUG
  cerr << __func__ << "(targetSize=" << targetSize << ")" << endl;
#endif

  str << source;
  str >> resetiosflags(ios::basefield);   // guess base from prefix
  str.exceptions(ios::failbit);

#if DEBUG
  cerr << "code=" << (int) type->code << endl;
#endif

  switch (type->code) {
    case 0x0001:     // bool
      {
        int16_t val;         // uint8_t is interpreted as char
        str >> val;
        if (val > 1 || val < 0) {
          throw ios::failure("Value out of range");
        }
        *(uint8_t *) target = val;
        break;
      }
    case 0x0002:     // int8
      {
        int16_t val;         // uint8_t is interpreted as char
        str >> val;
        if (val > 127 || val < -128) {
          throw ios::failure("Value out of range");
        }
        *(uint8_t *) target = val;
        break;
      }
    case 0x0003:     // int16
      {
        int16_t val;
        str >> val;
        *(int16_t *) target = cpu_to_le16(val);
        break;
      }
    case 0x0004:     // int32
      {
        int32_t val;
        str >> val;
        *(int32_t *) target = cpu_to_le32(val);
        break;
      }
    case 0x0005:     // uint8
      {
        uint16_t val;         // uint8_t is interpreted as char
        str >> val;
        if (val > 0xff) {
          throw ios::failure("Value out of range");
        }
        *(uint8_t *) target = val;
        break;
      }
    case 0x0006:     // uint16
      {
        uint16_t val;
        str >> val;
        *(uint16_t *) target = cpu_to_le16(val);
        break;
      }
    case 0x0007:     // uint32
      {
        uint32_t val;
        str >> val;
        *(uint32_t *) target = cpu_to_le32(val);
        break;
      }
    case 0x0008:     // float
      {
        float val;
        str >> val;
        *(uint32_t *) target =
          cpu_to_le32(*(uint32_t *) (void *) &val);
        break;
      }
    case 0x0009:     // string
    case 0x000a:     // octet_string
    case 0x000b:     // unicode_string
      dataSize = str.str().size();
      if (dataSize > targetSize) {
        stringstream err;
        err << "String too large ("
            << dataSize << " > " << targetSize << ")";
        throw SizeException(err.str());
      }
      str.read((char *) target, dataSize);
      break;
    case 0x0011:     // double
      {
        double val;
        str >> val;
        *(uint64_t *) target =
          cpu_to_le64(*(uint64_t *) (void *) &val);
        break;
      }
      break;
    case 0x0015:     // int64
      {
        int64_t val;
        str >> val;
        *(int64_t *) target = cpu_to_le64(val);
        break;
      }
      break;
    case 0x001b:     // uint64
      {
        uint64_t val;
        str >> val;
        *(uint64_t *) target = cpu_to_le64(val);
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
        stringstream err;
        err << "Non-native integer type " << type->name
            << " is not yet implemented.";
        throw runtime_error(err.str());
      }

    case 0xfffb:     // sm8
    case 0xfffc:     // sm16
    case 0xfffd:     // sm32
    case 0xfffe:     // sm64
      {
        stringstream err;
        err << "Sign-and-magitude types not yet"
          " implemented for input direction.";
        throw runtime_error(err.str());
      }

    default:
      {
        stringstream err;
        err << "Unknown data type 0x" << hex << type->code;
        throw runtime_error(err.str());
      }
  }

#if DEBUG
  printRawData(cerr, (const uint8_t *) target, dataSize);
#endif

  return dataSize;
}

/****************************************************************************/

void DataTypeHandler::outputData(
  ostream & o,
  const DataType * type,
  void * data,
  size_t dataSize
)
{
  uint16_t typeCode;

  if (type) {
    if (type->byteSize && dataSize != type->byteSize) {
      stringstream err;
      err << "Data type mismatch. Expected " << type->name
          << " with " << type->byteSize << " byte, but got "
          << dataSize << " byte.";
      throw SizeException(err.str());
    }
    typeCode = type->code;
  } else {
    typeCode = 0xffff;     // raw data
  }

  o << setfill('0');

  switch (typeCode) {
    case 0x0001:     // bool
      {
        int val = (int) *(int8_t *) data;
        o << "0x" << hex << setw(2) << val
          << " " << dec << val << endl;
      }
      break;
    case 0x0002:     // int8
      {
        int val = (int) *(int8_t *) data;
        o << "0x" << hex << setw(2) << val
          << " " << dec << val << endl;
      }
      break;
    case 0x0003:     // int16
      {
        int16_t val = le16_to_cpup(data);
        o << "0x" << hex << setw(4) << val
          << " " << dec << val << endl;
      }
      break;
    case 0x0004:     // int32
      {
        int32_t val = le32_to_cpup(data);
        o << "0x" << hex << setw(8) << val
          << " " << dec << val << endl;
      }
      break;
    case 0x0005:     // uint8
      {
        unsigned int val = (unsigned int) *(uint8_t *) data;
        o << "0x" << hex << setw(2) << val
          << " " << dec << val << endl;
      }
      break;
    case 0x0006:     // uint16
      {
        uint16_t val = le16_to_cpup(data);
        o << "0x" << hex << setw(4) << val
          << " " << dec << val << endl;
      }
      break;
    case 0x0007:     // uint32
      {
        uint32_t val = le32_to_cpup(data);
        o << "0x" << hex << setw(8) << val
          << " " << dec << val << endl;
      }
      break;
    case 0x0008:     // float
      {
        uint32_t val = le32_to_cpup(data);
        float fval = *(float *) (void *) &val;
        o << fval << endl;
      }
      break;
    case 0x0009:     // string
      o << string((const char *) data, dataSize) << endl;
      break;
    case 0x000a:     // octet_string
      o << string((const char *) data, dataSize) << flush;
      break;
    case 0x000b:     // unicode_string
                     // FIXME encoding
      o << string((const char *) data, dataSize) << endl;
      break;
    case 0x0011:     // double
      {
        uint64_t val = le64_to_cpup(data);
        double fval = *(double *) (void *) &val;
        o << fval << endl;
      }
      break;
    case 0x0015:     // int64
      {
        int64_t val = le64_to_cpup(data);
        o << "0x" << hex << setw(16) << val
          << " " << dec << val << endl;
      }
      break;
    case 0x001b:     // uint64
      {
        uint64_t val = le64_to_cpup(data);
        o << "0x" << hex << setw(16) << val
          << " " << dec << val << endl;
      }
      break;
    case 0xfffb:     // sm8
      {
        int8_t val = *(uint8_t *) data;
        int8_t smval = val < 0 ? (val & 0x7f) * -1 : val;

        o << "0x" << hex << setw(2) << (int) val
          << " " << dec << (int) smval << endl;
      }
      break;
    case 0xfffc:     // sm16
      {
        int16_t val = le16_to_cpup(data);
        int16_t smval = val < 0 ? (val & 0x7fff) * -1 : val;

        o << "0x" << hex << setw(4) << val
          << " " << dec << smval << endl;
      }
      break;
    case 0xfffd:     // sm32
      {
        int32_t val = le32_to_cpup(data);
        int32_t smval = val < 0 ? (val & 0x7fffffffUL) * -1 : val;

        o << "0x" << hex << setw(8) << val
          << " " << dec << smval << endl;
      }
      break;
    case 0xfffe:     // sm64
      {
        int64_t val = le64_to_cpup(data);
        int64_t smval =
          val < 0 ? (val & 0x7fffffffffffffffULL) * -1 : val;

        o << "0x" << hex << setw(16) << val
          << " " << dec << smval << endl;
      }
      break;

    default:
      printRawData(o, (const uint8_t *) data, dataSize);       // FIXME
      break;
  }
}

/****************************************************************************/

void DataTypeHandler::printRawData(
  ostream & o,
  const uint8_t * data,
  size_t size
)
{
  o << hex << setfill('0');
  while (size--) {
    o << "0x" << setw(2) << (unsigned int) *data++;
    if (size) {
      o << " ";
    }
  }
  o << endl;
}

/****************************************************************************/

const DataTypeHandler::DataType DataTypeHandler::dataTypes[] = {
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

/*****************************************************************************/
