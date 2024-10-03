/**
 * @file ring_buffer.cpp
 * @brief Class to emulate ring buffer
 */

#include "ring_buffer.hpp"

#include <stdlib.h>

#include <algorithm>
#include <cstring>

namespace s2e::utilities {

RingBuffer::RingBuffer(int buffer_size) : buffer_size_(buffer_size) {
  buffer_ = new byte[buffer_size];
  write_pointer_ = 0;
  read_pointer_ = 0;
}

RingBuffer::~RingBuffer() { delete[] buffer_; }

int RingBuffer::Write(const byte* buffer, const unsigned int offset, const unsigned int data_length) {
  unsigned int write_count = 0;
  while (write_count != data_length) {
    unsigned int write_len = std::min(buffer_size_ - write_pointer_, data_length - write_count);
    memcpy(&buffer_[write_pointer_], &buffer[offset + write_count], write_len);
    write_pointer_ = (write_pointer_ + write_len == buffer_size_) ? 0 : write_pointer_ + write_len;
    write_count += write_len;
  }

  return write_count;
}

int RingBuffer::Read(byte* buffer, const unsigned int offset, const unsigned int data_length) {
  unsigned int read_count = 0;
  // There are four behaviors depending on whether the RP overtakes the WP, or
  // whether all of the RP to the WP are requested by data_length.
  while (read_count != data_length && write_pointer_ != read_pointer_) {
    unsigned int read_len = (write_pointer_ > read_pointer_) ? std::min(write_pointer_ - read_pointer_, data_length - read_count)
                                                             : std::min(buffer_size_ - read_pointer_, data_length - read_count);
    memcpy(&buffer[offset + read_count], &buffer_[read_pointer_], read_len);
    read_pointer_ = (read_pointer_ + read_len == buffer_size_) ? 0 : read_pointer_ + read_len;
    read_count += read_len;
  }

  return read_count;
}

}  // namespace s2e::utilities
