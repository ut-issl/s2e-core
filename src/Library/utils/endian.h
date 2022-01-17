#pragma once

#include "ENDIAN_DEFINE.h" // for IS_LITTLE_ENDIAN
#include <stddef.h>
#include <stdint.h>

void *endian_memcpy(void *dst, const void *src, size_t count);