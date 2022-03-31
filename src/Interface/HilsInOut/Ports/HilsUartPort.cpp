#include "HilsUartPort.h"

HilsUartPort::HilsUartPort(const unsigned int port_id, const unsigned int baud_rate, const unsigned int tx_buffer_size,
                           const unsigned int rx_buffer_size)
    : kPortName(PortName(port_id)), baud_rate_(baud_rate), kTxBufferSize(tx_buffer_size), kRxBufferSize(rx_buffer_size) {
  // Allocate managed arrays.
  tx_buf_ = gcnew bytearray(kTxBufferSize);
  rx_buf_ = gcnew bytearray(kRxBufferSize);

  Initialize();
}

HilsUartPort::~HilsUartPort() {
  // Memory allocated by gcnew does not have to be explicitly deleted.
}

// Static method to convert from com port number to com port name.
std::string HilsUartPort::PortName(unsigned int port_id) { return "COM" + std::to_string(port_id); }

int HilsUartPort::Initialize() {
  try {
    // Initialize port
    // Type conversion from unmanaged string to managed string
    msclr::gcroot<System::String ^> port_name_managed = msclr::interop::marshal_as<System::String ^>(kPortName);
    port_ = gcnew System::IO::Ports::SerialPort(port_name_managed, baud_rate_);
    // TODO: set parameter
    port_->ReadTimeout = 10;   // [ms]
    port_->WriteTimeout = 10;  // [ms]
  } catch (System::Exception ^ e) {
    // baudrate must be larger than zero
    // port nameのチェックはここでは行われない
    System::Console::Write(e->Message);
    return -1;
  }

  int ret = OpenPort();

  return ret;
}

int HilsUartPort::ClosePort() {
  try {
    port_->Close();
  } catch (System::Exception ^ e) {
    System::Console::Write(e->Message);
    return -1;
  }
  return 0;
}

int HilsUartPort::OpenPort() {
  try {
    port_->Open();
    // TODO: Add enum for exception
    // Exception reference
    // https://docs.microsoft.com/en-us/dotnet/api/system.io.ports.serialport.open?view=netframework-4.7.2

  } catch (System::UnauthorizedAccessException ^ e) {
    // Access is denied to the port.
    //   or
    // The current process, or another process on the system, already has the
    // specified COM port open
    //   either by a SerialPort instance or in unmanaged code.
    System::Console::Write(e->Message);
    return -2;
  } catch (System::ArgumentOutOfRangeException ^ e) {
    // One or more of the properties for this instance are invalid.
    System::Console::Write(e->Message);
    return -3;
  } catch (System::ArgumentException ^ e) {
    // The port name does not begin with "COM".
    //   or
    // The file type of the port is not supported.
    System::Console::Write(e->Message);
    return -4;
  } catch (System::IO::IOException ^ e) {
    // The port is in an invalid state.
    System::Console::Write(e->Message);
    return -5;
  }
  return 0;  // Success !!
}

int HilsUartPort::WriteTx(const unsigned char* buffer, int offset, int count) {
  unsigned char* buffer_tmp = new unsigned char[count];
  memcpy(buffer_tmp, buffer + offset,
         count);  // const unsigned char* -> unsigned char*
  // Marshal::Copy : Copies data from an unmanaged memory pointer to a managed
  // array.
  System::Runtime::InteropServices::Marshal::Copy((System::IntPtr)(buffer_tmp), tx_buf_, 0,
                                                  count);  // unsigned char* -> System::IntPtr
  delete[] buffer_tmp;
  try {
    port_->Write(tx_buf_, 0, count);
  } catch (System::Exception ^ e) {
    System::Console::Write(e->Message);
    return -1;
  }
  return 0;
}

int HilsUartPort::ReadRx(unsigned char* buffer, int offset, int count) {
  try {
    int received_bytes = port_->Read(rx_buf_, 0, count);
    // Marshal::Copy : Copies data from a managed array to an unmanaged memory
    // pointer.
    System::Runtime::InteropServices::Marshal::Copy(rx_buf_, 0, (System::IntPtr)(buffer + offset), count);
    return received_bytes;
    // TODO: Add enum for exception
  } catch (System::TimeoutException ^ e) {
    // No bytes were available to read.
    return -1;
  } catch (System::Exception ^ e) {
    System::Console::Write(e->Message);
    return -2;
  }
}

int HilsUartPort::GetBytesToRead() {
  int bytes_to_read;
  try {
    bytes_to_read = port_->BytesToRead;
  } catch (System::Exception ^ e) {
    // Port is not open
    System::Console::Write(e->Message);
    return -1;
  }
  return bytes_to_read;
}

int HilsUartPort::DiscardInBuffer() {
  try {
    port_->DiscardInBuffer();
  } catch (System::Exception ^ e) {
    System::Console::Write(e->Message);
    return -1;
  }
  return 0;
}

int HilsUartPort::DiscardOutBuffer() {
  try {
    port_->DiscardOutBuffer();
  } catch (System::Exception ^ e) {
    System::Console::Write(e->Message);
    return -1;
  }
  return 0;
}
