/**
 * @file uart_port.cpp
 * @brief Class to emulate UART communication port
 */

#include "uart_port.hpp"

UartPort::UartPort() : UartPort(kDefaultBufferSize, kDefaultBufferSize) {}

UartPort::UartPort(const unsigned int rx_buffer_size, const unsigned int tx_buffer_size) {
  unsigned int checked_rx_buffer_size = rx_buffer_size;
  unsigned int checked_tx_buffer_size = tx_buffer_size;
  if (rx_buffer_size <= 0) checked_rx_buffer_size = kDefaultBufferSize;
  if (tx_buffer_size <= 0) checked_tx_buffer_size = kDefaultBufferSize;
  rx_buffer_ = new RingBuffer(checked_rx_buffer_size);
  tx_buffer_ = new RingBuffer(checked_tx_buffer_size);
}

UartPort::~UartPort() {
  delete rx_buffer_;
  delete tx_buffer_;
}

int UartPort::WriteTx(const unsigned char* buffer, const unsigned int offset, const unsigned int data_length) {
  return tx_buffer_->Write(buffer, offset, data_length);
}

int UartPort::WriteRx(const unsigned char* buffer, const unsigned int offset, const unsigned int data_length) {
  return rx_buffer_->Write(buffer, offset, data_length);
}

int UartPort::ReadTx(unsigned char* buffer, const unsigned int offset, const unsigned int data_length) {
  return tx_buffer_->Read(buffer, offset, data_length);
}

int UartPort::ReadRx(unsigned char* buffer, const unsigned int offset, const unsigned int data_length) {
  return rx_buffer_->Read(buffer, offset, data_length);
}