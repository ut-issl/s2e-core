/**
 * @file endian.h
 * @brief Function to consider the endian
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "ENDIAN_DEFINE.h"  // for IS_LITTLE_ENDIAN

/**
 * @fn endian_memcpy
 * @brief Memory copy considering endian
 * @param [out] dst: Copy destination
 * @param [in] src: Copy source
 * @param [in] count: Copy data size
 */
void *endian_memcpy(void *dst, const void *src, size_t count);