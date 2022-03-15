#pragma once
#include <msclr/gcroot.h>
#include <msclr/marshal_cppstd.h>

#include <string>

// SerialPort Class Reference:
// https://docs.microsoft.com/en-us/dotnet/api/system.io.ports.serialport?view=netframework-4.7.2
// System::Byte: an 8-bit unsigned integer
typedef cli::array<System::Byte> bytearray;

class HilsUartPort {
 public:
  HilsUartPort(const unsigned int port_id, const unsigned int baud_rate,
               const unsigned int tx_buffer_size,
               const unsigned int rx_buffer_size);
  ~HilsUartPort();
  int OpenPort();
  int ClosePort();
  int WriteTx(const unsigned char* buffer, int offset, int count);
  int ReadRx(unsigned char* buffer, int offset, int count);
  int GetBytesToRead();

 private:
  const unsigned int kTxBufferSize;
  const unsigned int kRxBufferSize;
  const std::string kPortName;
  static std::string PortName(unsigned int port_id);
  int Initialize();
  int DiscardInBuffer();
  int DiscardOutBuffer();
  unsigned int baud_rate_;  // [baud] ex. 9600, 115200
  // gcroot is the type-safe wrapper template to refer to a CLR object from the
  // c++ heap reference:
  // https://docs.microsoft.com/en-us/cpp/dotnet/how-to-declare-handles-in-native-types?view=msvc-160
  msclr::gcroot<System::IO::Ports::SerialPort ^> port_;
  msclr::gcroot<bytearray ^> tx_buf_;
  msclr::gcroot<bytearray ^> rx_buf_;
};
