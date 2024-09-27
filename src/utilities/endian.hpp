/**
 * @file endian.hpp
 * @brief Function to consider the endian
 */

#ifndef S2E_LIBRARY_UTILITIES_ENDIAN_HPP_
#define S2E_LIBRARY_UTILITIES_ENDIAN_HPP_

#include <stddef.h>
#include <stdint.h>

#include "endian_define.hpp"  // for IS_LITTLE_ENDIAN

namespace s2e::utilities {

/**
 * @fn endian_memcpy
 * @brief Memory copy considering endian
 * @param [out] dst: Copy destination
 * @param [in] src: Copy source
 * @param [in] count: Copy data size
 */
void *endian_memcpy(void *dst, const void *src, size_t count);

} // namespace s2e::utilities

#endif  // S2E_LIBRARY_UTILITIES_ENDIAN_HPP_
