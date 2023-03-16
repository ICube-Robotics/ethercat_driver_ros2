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

#ifndef ETHERCAT_GENERIC_PLUGINS__CIA402_COMMON_DEFS_HPP_
#define ETHERCAT_GENERIC_PLUGINS__CIA402_COMMON_DEFS_HPP_

#define CiA402D_RPDO_CONTROLWORD  ((uint16_t) 0x6040)
#define CiA402D_RPDO_POSITION  ((uint16_t) 0x607a)
#define CiA402D_RPDO_VELOCITY  ((uint16_t) 0x60ff)
#define CiA402D_RPDO_EFFORT  ((uint16_t) 0x6071)
#define CiA402D_RPDO_MODE_OF_OPERATION  ((uint16_t) 0x6060)

#define CiA402D_TPDO_POSITION ((uint16_t) 0x6064)
#define CiA402D_TPDO_STATUSWORD  ((uint16_t) 0x6041)
#define CiA402D_TPDO_MODE_OF_OPERATION_DISPLAY  ((uint16_t) 0x6061)

#include <map>
#include <string>

enum DeviceState
{
  STATE_UNDEFINED = 0,
  STATE_START = 1,
  STATE_NOT_READY_TO_SWITCH_ON,
  STATE_SWITCH_ON_DISABLED,
  STATE_READY_TO_SWITCH_ON,
  STATE_SWITCH_ON,
  STATE_OPERATION_ENABLED,
  STATE_QUICK_STOP_ACTIVE,
  STATE_FAULT_REACTION_ACTIVE,
  STATE_FAULT
};

enum ModeOfOperation
{
  MODE_NO_MODE                = 0,
  MODE_PROFILED_POSITION      = 1,
  MODE_PROFILED_VELOCITY      = 3,
  MODE_PROFILED_TORQUE        = 4,
  MODE_HOMING                 = 6,
  MODE_INTERPOLATED_POSITION  = 7,
  MODE_CYCLIC_SYNC_POSITION   = 8,
  MODE_CYCLIC_SYNC_VELOCITY  = 9,
  MODE_CYCLIC_SYNC_TORQUE     = 10
};

const std::map<DeviceState, std::string> DEVICE_STATE_STR = {
  {STATE_START, "Start"},
  {STATE_NOT_READY_TO_SWITCH_ON, "Not Ready to Switch On"},
  {STATE_SWITCH_ON_DISABLED, "Switch on Disabled"},
  {STATE_READY_TO_SWITCH_ON, "Ready to Switch On"},
  {STATE_SWITCH_ON, "Switch On"},
  {STATE_OPERATION_ENABLED, "Operation Enabled"},
  {STATE_QUICK_STOP_ACTIVE, "Quick Stop Active"},
  {STATE_FAULT_REACTION_ACTIVE, "Fault Reaction Active"},
  {STATE_FAULT, "Fault"},
  {STATE_UNDEFINED, "Undefined State"}
};

#endif  // ETHERCAT_GENERIC_PLUGINS__CIA402_COMMON_DEFS_HPP_
