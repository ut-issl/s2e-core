#include "endian.h"

#include <stdlib.h>

void *endian_memcpy(void *dst, const void *src, size_t size) {
#ifdef IS_LITTLE_ENDIAN
  uint8_t *src_ = (uint8_t *)src;
  uint8_t *dst_ = (uint8_t *)dst;
  size_t i;

  size--;

  for (i = 0; i <= size; i++) {
    *(dst_ + (size - i)) = *(src_ + i);
  }

  return dst;
#else
  return memcpy(dst, src, size);  // 基本ここは使われないはず。
#endif  // IS_LITTLE_ENDIAN
}