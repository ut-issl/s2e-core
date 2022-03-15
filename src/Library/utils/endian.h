#pragma once

#include <stddef.h>
#include <stdint.h>

#include "ENDIAN_DEFINE.h"  // for IS_LITTLE_ENDIAN

void *endian_memcpy(void *dst, const void *src, size_t count);