// Copyright 2024 ICUBE Laboratory, University of Strasbourg
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
// Author: Manuel YGUEL (yguel.robotics@gmail.com)

#ifndef ETHERCAT_INTERFACE__EC_TRANSFER_HPP_
#define ETHERCAT_INTERFACE__EC_TRANSFER_HPP_

#include <cstdint>
#include <vector>

namespace ethercat_interface
{

// Forward declaration to avoid circular dependency
struct DomainInfo;

struct EcTransferInfo
{
  const DomainInfo * input_domain;
  const DomainInfo * output_domain;

  /** Pointer into the input process domain, equal to
   * domain process data pointer + the offset
   * defined in the ec_pdo_entry_reg_t data structure
   * of a domain_regs array in a DomainInfo data structure.
   */
  uint8_t * in_ptr;

  /** Pointer into the output process domain, equal to
   * domain process data pointer + the offset
   * defined in the ec_pdo_entry_reg_t data structure
   * of a domain_regs array in a DomainInfo data structure.
   */
  uint8_t * out_ptr;

  /** Number of octets of the exchange data. */
  size_t size;
};

}  // namespace ethercat_interface

#endif  // ETHERCAT_INTERFACE__EC_TRANSFER_HPP_
