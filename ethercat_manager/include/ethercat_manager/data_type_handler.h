/*****************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2006-2009  Florian Pose, Ingenieurgemeinschaft IgH
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

#ifndef ETHERCAT_MANAGER__DATA_TYPE_HANDLER_H_
#define ETHERCAT_MANAGER__DATA_TYPE_HANDLER_H_

/****************************************************************************/

#include <stdint.h>
#include <string>
#include <stdexcept>
#include <ostream>

/****************************************************************************/

class DataTypeHandler
{
public:
  DataTypeHandler();

  struct DataType
  {
    const char * name;
    uint16_t code;
    size_t byteSize;
  };

  static std::string typeInfo();

  static const DataType * findDataType(const std::string &);
  static const DataType * findDataType(uint16_t);
  static size_t interpretAsType(
    const DataType *, const std::string &,
    void *, size_t);

  class SizeException:
  public std::runtime_error
  {
public:
    explicit SizeException(const std::string & msg)
      : runtime_error(msg) {
    }
  };

  static void outputData(
    std::ostream &, const DataType *,
    void *, size_t);
  static void printRawData(std::ostream &, const uint8_t *, size_t);

private:
  static const DataType dataTypes[];
};

/****************************************************************************/

#endif  // ETHERCAT_MANAGER__DATA_TYPE_HANDLER_H_
