/**
 * @file endian.cpp
 * @brief Function to consider the endian
 */

#include "endian.hpp"

#include <stdlib.h>

namespace s2e::utilities {

void *endian_memcpy(void *dst, const void *src, size_t size) {
#ifdef IS_LITTLE_ENDIAN
  uint8_t *src_ = (uint8_t *)src;
  uint8_t *dst_ = (uint8_t *)dst;
  size_t i;

  if (size > 0) {
    size--;
  }

  for (i = 0; i <= size; i++) {
    *(dst_ + (size - i)) = *(src_ + i);
  }

  return dst;
#else
  return memcpy(dst, src, size);
#endif  // IS_LITTLE_ENDIAN
}

} // namespace s2e::utilities
