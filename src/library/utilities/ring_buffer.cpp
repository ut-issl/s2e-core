/**
 * @file ring_buffer.cpp
 * @brief Class to emulate ring buffer
 */

#include "ring_buffer.hpp"

#include <stdlib.h>

#include <algorithm>
#include <cstring>

RingBuffer::RingBuffer(int bufSize) : buffer_size_(bufSize) {
  buffer_ = new byte[bufSize];
  write_pointer = 0;
  read_pointer = 0;
}

RingBuffer::~RingBuffer() { delete[] buffer_; }

int RingBuffer::Write(byte* buffer, int offset, int count) {
  int write_count = 0;
  while (write_count != count) {
    int write_len = std::min(buffer_size_ - write_pointer, count - write_count);
    memcpy(&buffer_[write_pointer], &buffer[offset + write_count], write_len);
    write_pointer = (write_pointer + write_len == buffer_size_) ? 0 : write_pointer + write_len;
    write_count += write_len;
  }

  return write_count;
}

int RingBuffer::Read(byte* buffer, int offset, int count) {
  int read_count = 0;
  // There are four behaviors depending on whether the RP overtakes the WP, or
  // whether all of the RP to the WP are requested by count.
  while (read_count != count && write_pointer != read_pointer) {
    int read_len = (write_pointer > read_pointer) ? std::min(write_pointer - read_pointer, count - read_count)
                                                  : std::min(buffer_size_ - read_pointer, count - read_count);
    memcpy(&buffer[offset + read_count], &buffer_[read_pointer], read_len);
    read_pointer = (read_pointer + read_len == buffer_size_) ? 0 : read_pointer + read_len;
    read_count += read_len;
  }

  return read_count;
}
