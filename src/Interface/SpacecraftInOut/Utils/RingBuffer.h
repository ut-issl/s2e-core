#pragma once

typedef unsigned char byte;

class RingBuffer {
 public:
  RingBuffer(int bufSize);
  ~RingBuffer();
  int Write(byte* buffer, int offset, int count);
  int Read(byte* buffer, int offset, int count);

 private:
  const int kBufferSize;
  byte* buf_;
  int rp_, wp_;
};
