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

#ifndef ETHERCAT_INTERFACE__EC_BUFFER_TOOLS_H_
#define ETHERCAT_INTERFACE__EC_BUFFER_TOOLS_H_

#include <asm/byteorder.h>
/******************************************************************************
 * Bitwise read/write macros
 *****************************************************************************/

/** Read a certain bit of an Buffer data byte.
 *
 * \param DATA Buffer data pointer
 * \param POS bit position
 */
#define EC_READ_BIT(DATA, POS) ((*((uint8_t *) (DATA)) >> (POS)) & 0x01)

/** Write a certain bit of an Buffer data byte.
 *
 * \param DATA Buffer data pointer
 * \param POS bit position
 * \param VAL new bit value
 */
#define write_BIT(DATA, POS, VAL) \
  do { \
    if (VAL) *((uint8_t *) (DATA)) |= (1 << (POS)); \
    else * ((uint8_t *) (DATA)) &= ~(1 << (POS)); \
  } while (0)

/******************************************************************************
 * Byte-swapping functions for user space
 *****************************************************************************/

#ifndef __KERNEL__

#if __BYTE_ORDER == __LITTLE_ENDIAN

#define le16_to_cpu(x) x
#define le32_to_cpu(x) x
#define le64_to_cpu(x) x

#define cpu_to_le16(x) x
#define cpu_to_le32(x) x
#define cpu_to_le64(x) x

#elif __BYTE_ORDER == __BIG_ENDIAN

#define swap16(x) \
  ((uint16_t)( \
    (((uint16_t)(x) & 0x00ffU) << 8) | \
    (((uint16_t)(x) & 0xff00U) >> 8) ))
#define swap32(x) \
  ((uint32_t)( \
    (((uint32_t)(x) & 0x000000ffUL) << 24) | \
    (((uint32_t)(x) & 0x0000ff00UL) << 8) | \
    (((uint32_t)(x) & 0x00ff0000UL) >> 8) | \
    (((uint32_t)(x) & 0xff000000UL) >> 24) ))
#define swap64(x) \
  ((uint64_t)( \
    (((uint64_t)(x) & 0x00000000000000ffULL) << 56) | \
    (((uint64_t)(x) & 0x000000000000ff00ULL) << 40) | \
    (((uint64_t)(x) & 0x0000000000ff0000ULL) << 24) | \
    (((uint64_t)(x) & 0x00000000ff000000ULL) << 8) | \
    (((uint64_t)(x) & 0x000000ff00000000ULL) >> 8) | \
    (((uint64_t)(x) & 0x0000ff0000000000ULL) >> 24) | \
    (((uint64_t)(x) & 0x00ff000000000000ULL) >> 40) | \
    (((uint64_t)(x) & 0xff00000000000000ULL) >> 56) ))

#define le16_to_cpu(x) swap16(x)
#define le32_to_cpu(x) swap32(x)
#define le64_to_cpu(x) swap64(x)

#define cpu_to_le16(x) swap16(x)
#define cpu_to_le32(x) swap32(x)
#define cpu_to_le64(x) swap64(x)

#endif

#define le16_to_cpup(x) le16_to_cpu(*((uint16_t *)(x)))
#define le32_to_cpup(x) le32_to_cpu(*((uint32_t *)(x)))
#define le64_to_cpup(x) le64_to_cpu(*((uint64_t *)(x)))

#endif /* ifndef __KERNEL__ */

/******************************************************************************
 * Read macros
 *****************************************************************************/

/** Read an 8-bit unsigned value from Buffer data.
 *
 * \return Buffer data value
 */
#define read_u8(DATA) \
  ((uint8_t) *((uint8_t *) (DATA)))

/** Read an 8-bit signed value from Buffer data.
 *
 * \param DATA Buffer data pointer
 * \return Buffer data value
 */
#define read_s8(DATA) \
  ((int8_t) *((uint8_t *) (DATA)))

/** Read a 16-bit unsigned value from Buffer data.
 *
 * \param DATA Buffer data pointer
 * \return Buffer data value
 */
#define read_u16(DATA) \
  ((uint16_t) le16_to_cpup((void *) (DATA)))

/** Read a 16-bit signed value from Buffer data.
 *
 * \param DATA Buffer data pointer
 * \return Buffer data value
 */
#define read_s16(DATA) \
  ((int16_t) le16_to_cpup((void *) (DATA)))

/** Read a 32-bit unsigned value from Buffer data.
 *
 * \param DATA Buffer data pointer
 * \return Buffer data value
 */
#define read_u32(DATA) \
  ((uint32_t) le32_to_cpup((void *) (DATA)))

/** Read a 32-bit signed value from Buffer data.
 *
 * \param DATA Buffer data pointer
 * \return Buffer data value
 */
#define read_s32(DATA) \
  ((int32_t) le32_to_cpup((void *) (DATA)))

/** Read a 64-bit unsigned value from Buffer data.
 *
 * \param DATA Buffer data pointer
 * \return Buffer data value
 */
#define read_u64(DATA) \
  ((uint64_t) le64_to_cpup((void *) (DATA)))

/** Read a 64-bit signed value from Buffer data.
 *
 * \param DATA Buffer data pointer
 * \return Buffer data value
 */
#define read_s64(DATA) \
  ((int64_t) le64_to_cpup((void *) (DATA)))

/******************************************************************************
 * Write macros
 *****************************************************************************/

/** Write an 8-bit unsigned value to Buffer data.
 *
 * \param DATA Buffer data pointer
 * \param VAL new value
 */
#define write_u8(DATA, VAL) \
  do { \
    *((uint8_t *)(DATA)) = ((uint8_t) (VAL)); \
  } while (0)

/** Write an 8-bit signed value to Buffer data.
 *
 * \param DATA Buffer data pointer
 * \param VAL new value
 */
#define write_s8(DATA, VAL) write_u8(DATA, VAL)

/** Write a 16-bit unsigned value to Buffer data.
 *
 * \param DATA Buffer data pointer
 * \param VAL new value
 */
#define write_u16(DATA, VAL) \
  do { \
    *((uint16_t *) (DATA)) = cpu_to_le16((uint16_t) (VAL)); \
  } while (0)

/** Write a 16-bit signed value to Buffer data.
 *
 * \param DATA Buffer data pointer
 * \param VAL new value
 */
#define write_s16(DATA, VAL) write_u16(DATA, VAL)

/** Write a 32-bit unsigned value to Buffer data.
 *
 * \param DATA Buffer data pointer
 * \param VAL new value
 */
#define write_u32(DATA, VAL) \
  do { \
    *((uint32_t *) (DATA)) = cpu_to_le32((uint32_t) (VAL)); \
  } while (0)

/** Write a 32-bit signed value to Buffer data.
 *
 * \param DATA Buffer data pointer
 * \param VAL new value
 */
#define write_s32(DATA, VAL) write_u32(DATA, VAL)

/** Write a 64-bit unsigned value to Buffer data.
 *
 * \param DATA Buffer data pointer
 * \param VAL new value
 */
#define write_u64(DATA, VAL) \
  do { \
    *((uint64_t *) (DATA)) = cpu_to_le64((uint64_t) (VAL)); \
  } while (0)

/** Write a 64-bit signed value to Buffer data.
 *
 * \param DATA Buffer data pointer
 * \param VAL new value
 */
#define write_s64(DATA, VAL) write_u64(DATA, VAL)

#endif  // ETHERCAT_INTERFACE__EC_BUFFER_TOOLS_H_
