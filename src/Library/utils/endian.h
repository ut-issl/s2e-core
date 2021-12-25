#pragma once

#include <stdint.h>
#include <stddef.h>
#include "ENDIAN_DEFINE.h" // for IS_LITTLE_ENDIAN

void *endian_memcpy(void *dst, const void *src, size_t count);