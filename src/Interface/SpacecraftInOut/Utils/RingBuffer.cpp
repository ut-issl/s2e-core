#include "RingBuffer.h"
#include <algorithm>
#include <cstring>
#include <stdlib.h>

RingBuffer::RingBuffer(int bufSize) : kBufferSize(bufSize) {
  buf_ = new byte[bufSize];
  wp_ = 0;
  rp_ = 0;
}

RingBuffer::~RingBuffer() { delete[] buf_; }

// Write count bytes from offset byte of the buffer to the RingBuffer
// Returns the number of bytes written
int RingBuffer::Write(byte *buffer, int offset, int count) {
  int write_count = 0;
  while (write_count != count) {
    int write_len = std::min(kBufferSize - wp_, count - write_count);
    memcpy(&buf_[wp_], &buffer[offset + write_count], write_len);
    wp_ = (wp_ + write_len == kBufferSize) ? 0 : wp_ + write_len;
    write_count += write_len;
  }

  return write_count;
}

// Read only count bytes from the offset byte of the buffer
// Return the number of bytes read
int RingBuffer::Read(byte *buffer, int offset, int count) {
  int read_count = 0;
  // There are four behaviors depending on whether the RP overtakes the WP, or
  // whether all of the RP to the WP are requested by count.
  // (RPがWPを追い越しているか、countでRPからWPまで全て要求されているかどうかにより、4通りの挙動)
  while (read_count != count && wp_ != rp_) {
    int read_len = (wp_ > rp_)
                       ? std::min(wp_ - rp_, count - read_count)
                       : std::min(kBufferSize - rp_, count - read_count);
    memcpy(&buffer[offset + read_count], &buf_[rp_], read_len);
    rp_ = (rp_ + read_len == kBufferSize) ? 0 : rp_ + read_len;
    read_count += read_len;
  }

  return read_count;
}
