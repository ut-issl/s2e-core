/**
 * @file ComPortInterface.cpp
 * @brief Class to manage PC's COM port
 * @details Currently, this feature supports Windows Visual Studio only.(FIXME)
 * Reference: https://docs.microsoft.com/ja-jp/dotnet/api/system.io.ports.serialport?view=netframework-4.7.2
 */

#include "ComPortInterface.h"

ComPortInterface::ComPortInterface(int port_id, int baudrate, unsigned int tx_buffer_size, unsigned int rx_buffer_size)
    : kPortId(port_id), kPortName(PortName(port_id)), kBaudRate(baudrate), kTxBufferSize(tx_buffer_size), kRxBufferSize(rx_buffer_size) {
  // Allocate managed memory
  tx_buf_ = gcnew bytearray(kTxBufferSize);
  rx_buf_ = gcnew bytearray(kRxBufferSize);

  // int ret = Initialize(); // TODO: Consider to call the Initialize function here.
}

ComPortInterface::~ComPortInterface() {
  // We don't need to delete the memory allocated by gcnew
}

std::string ComPortInterface::PortName(int port_id) { return "COM" + std::to_string(port_id); }

int ComPortInterface::Initialize() {
  try {
    // Initial setting
    port_ = gcnew IO::Ports::SerialPort(msclr::interop::marshal_as<String ^>(kPortName), kBaudRate);
  } catch (IO::IOException ^ e) {
    // Fault initial setting
    return -1;
  }

  // Open the port
  try {
    OpenPort();
    // Reference of errors：
    // https://docs.microsoft.com/en-us/dotnet/api/system.io.ports.serialport.open?view=netframework-4.7.2
  } catch (UnauthorizedAccessException ^ e) {
    // Access is denied to the port. or
    // The current process, or another process on the system, already has the
    // specified COM port open either by a SerialPort instance or in unmanaged code.
    return -2;
  } catch (ArgumentOutOfRangeException ^ e) {
    // One or more of the properties for this instance are invalid.
    return -3;
  } catch (ArgumentException ^ e) {
    // The port name does not begin with "COM". or
    // The file type of the port is not supported.
    return -4;
  } catch (IO::IOException ^ e) {
    // The port is in an invalid state.
    return -5;
  }

  return 0;  // Success!!
}

int ComPortInterface::Finalize() {
  try {
    port_->Close();
  } catch (Exception ^ e) {
    return -1;
  }
  return 0;
}

int ComPortInterface::OpenPort() {
  try {
    port_->Open();
  } catch (Exception ^ e) {
    return -1;
  }
  return 0;
}

int ComPortInterface::Send(unsigned char *buffer, size_t offset, size_t count) {
  Marshal::Copy((IntPtr)(buffer + offset), tx_buf_, 0, count);  // Copy to managed memory
  try {
    port_->Write(tx_buf_, 0, count);
  } catch (Exception ^ e) {
    return -1;
  }

  return 0;
}

int ComPortInterface::Receive(unsigned char *buffer, size_t offset, size_t count) {
  try {
    int received_bytes = port_->Read(rx_buf_, 0, count);
    Marshal::Copy(rx_buf_, 0, (IntPtr)(buffer + offset), count);  // Copy from managed memory
    return received_bytes;
  } catch (Exception ^ e) {
    return 0;
  }
}

int ComPortInterface::BytesToRead() {
  int bytes2read;
  try {
    bytes2read = port_->BytesToRead;
  } catch (Exception ^ e) {
    // Port is not opened
    return -1;
  }
  return bytes2read;  // port_->BytesToRead;
}

int ComPortInterface::DiscardInBuffer() {
  try {
    port_->DiscardInBuffer();
  } catch (Exception ^ e) {
    return -1;
  }
  return 0;
}
