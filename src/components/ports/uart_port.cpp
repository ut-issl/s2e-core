/**
 * @file uart_port.cpp
 * @brief Class to emulate UART communication port
 */

#include "uart_port.hpp"

SCIPort::SCIPort() : SCIPort(kDefaultBufferSize, kDefaultBufferSize) {}

SCIPort::SCIPort(int rx_buffer_size, int tx_buffer_size) {
  if (rx_buffer_size <= 0) rx_buffer_size = kDefaultBufferSize;
  if (tx_buffer_size <= 0) tx_buffer_size = kDefaultBufferSize;
  rxb_ = new RingBuffer(rx_buffer_size);
  txb_ = new RingBuffer(tx_buffer_size);
}

SCIPort::~SCIPort() {
  delete rxb_;
  delete txb_;
}

int SCIPort::WriteTx(unsigned char* buffer, int offset, int count) { return txb_->Write(buffer, offset, count); }

int SCIPort::WriteRx(unsigned char* buffer, int offset, int count) { return rxb_->Write(buffer, offset, count); }

int SCIPort::ReadTx(unsigned char* buffer, int offset, int count) { return txb_->Read(buffer, offset, count); }

int SCIPort::ReadRx(unsigned char* buffer, int offset, int count) { return rxb_->Read(buffer, offset, count); }