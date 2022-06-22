// Copyright 2022 ICUBE Laboratory, University of Strasbourg
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

#ifndef ETHERCAT_PLUGINS__COMMONDEFS_HPP_
#define ETHERCAT_PLUGINS__COMMONDEFS_HPP_

#define IO_WAIT ((uint8_t)0x00)
#define IO_NOTOK ((uint8_t)0x01)
#define IO_OK ((uint8_t)0x02)

#define CAL_SUCESS ((uint8_t)0x01)
#define CAL_DONE ((uint8_t)0x02)
#define CAL_ERROR ((uint8_t)0x03)
#define CAL_FINISHED ((uint8_t)0x04)

#define MODE_INIT ((uint8_t)0x00)
#define MODE_SEND_REQUEST ((uint8_t)0x01)
#define MODE_WAIT_REPLY ((uint8_t)0x02)
#define MODE_DO_CAL ((uint8_t)0x03)
#define MODE_CHECK_POS ((uint8_t)0x04)
#define MODE_GOTO_BACKPOS ((uint8_t)0x05)
#define MODE_WRITE_POS ((uint8_t)0x06)
#define MODE_SEARCH_LIMIT ((uint8_t)0x07)
#define MODE_SEARCH_TRANSITION ((uint8_t)0x08)
#define MODE_WAIT_DEAD_TIME ((uint8_t)0x09)

#define NULL_CMD ((uint16_t)0x00)
#define WRITE_REQUEST ((uint16_t)0x01)

#define OPERATION_ENABLED ((uint16_t)0x01)
#define OPERATION_DISABLED ((uint16_t)0x02)
#define CALIBRATION ((uint16_t)0x03)
#define CALIBRATION_INTERRUPT ((uint16_t)0x04)
#define CALIBRATION_END_SUCESS ((uint16_t)0x05)
#define CALIBRATION_REQUEST ((uint16_t)0x06)
#define FAULT_RESET ((uint16_t)0x07)
#define CALIBRATION_END_NOSUCESS ((uint16_t)0x08)
#define WRITE_HOMING_POSITION ((uint16_t)0x09)

#endif  // ETHERCAT_PLUGINS__COMMONDEFS_HPP_
