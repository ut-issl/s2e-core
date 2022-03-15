#include "SCIPort.h"

SCIPort::SCIPort() : SCIPort(kDefaultBufferSize, kDefaultBufferSize) {}

SCIPort::SCIPort(int rx_buf_size, int tx_buf_size) {
  if (rx_buf_size <= 0) rx_buf_size = kDefaultBufferSize;
  if (tx_buf_size <= 0) tx_buf_size = kDefaultBufferSize;
  rxb_ = new RingBuffer(rx_buf_size);
  txb_ = new RingBuffer(tx_buf_size);
}

SCIPort::~SCIPort() {
  delete rxb_;
  delete txb_;
}

int SCIPort::WriteTx(unsigned char* buffer, int offset, int count) {
  return txb_->Write(buffer, offset, count);
}

int SCIPort::WriteRx(unsigned char* buffer, int offset, int count) {
  return rxb_->Write(buffer, offset, count);
}

int SCIPort::ReadTx(unsigned char* buffer, int offset, int count) {
  return txb_->Read(buffer, offset, count);
}

int SCIPort::ReadRx(unsigned char* buffer, int offset, int count) {
  return rxb_->Read(buffer, offset, count);
}