#pragma once
// For multi-platform support, we have to modify the dependencies
#include <Component/Abstract/ITickable.h>
#include <msclr/gcroot.h>
#include <msclr/marshal_cppstd.h>

#include <string>

// SerialPort Classリファレンス：
// https://docs.microsoft.com/ja-jp/dotnet/api/system.io.ports.serialport?view=netframework-4.7.2

using namespace System;
using namespace System::Runtime::InteropServices;
typedef cli::array<Byte> bytearray;

class ComPortInterface {
 public:
  ComPortInterface(int port_id, int baudrate, unsigned int tx_buffer_size, unsigned int rx_buffer_size);
  ~ComPortInterface();
  static std::string PortName(int port_id);
  int Initialize();
  int Finalize();
  int Send(unsigned char *buffer, size_t offset, size_t count);
  int Receive(unsigned char *buffer, size_t offset, size_t count);
  int DiscardInBuffer();
  const size_t kTxBufferSize;
  const size_t kRxBufferSize;
  int BytesToRead();
  const std::string kPortName;
  const int kPortId;
  const int kBaudRate;

 private:
  int OpenPort();
  msclr::gcroot<IO::Ports::SerialPort ^> port_;
  // Byte tx_buf_[kTxBufferSize];
  // Byte rx_buf_[kRxBufferSize];
  msclr::gcroot<bytearray ^> tx_buf_;
  msclr::gcroot<bytearray ^> rx_buf_;
};
