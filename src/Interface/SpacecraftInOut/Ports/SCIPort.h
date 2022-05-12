#pragma once

#include "../Utils/RingBuffer.h"

/**
 * SCI(Serial Communication Interface) port.
 * Compatible with anything that performs data communication (UART, I2C, SPI).
 * The distinction of the area should be done where the upper port ID is
 * assigned
 */
class SCIPort {
 public:
  SCIPort();
  SCIPort(int rx_buf_size, int tx_buf_size);
  ~SCIPort();
  int WriteTx(unsigned char* buffer, int offset, int count);
  int WriteRx(unsigned char* buffer, int offset, int count);
  int ReadTx(unsigned char* buffer, int offset, int count);
  int ReadRx(unsigned char* buffer, int offset, int count);

 private:
  const static int kDefaultBufferSize = 1024;
  RingBuffer* rxb_;  /// Reception from the viewpoint of on-board software
                     /// (transmission from simulated component)
  RingBuffer* txb_;  /// Transmission from the viewpoint of the installed
                     /// software (reception from the simulated component)
};
