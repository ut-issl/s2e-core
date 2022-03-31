#include "ComPortInterface.h"

ComPortInterface::ComPortInterface(int port_id, int baudrate, unsigned int tx_buffer_size, unsigned int rx_buffer_size)
    : kPortId(port_id), kPortName(PortName(port_id)), kBaudRate(baudrate), kTxBufferSize(tx_buffer_size), kRxBufferSize(rx_buffer_size) {
  // Managed配列を確保
  tx_buf_ = gcnew bytearray(kTxBufferSize);
  rx_buf_ = gcnew bytearray(kRxBufferSize);

  // TODO: コンストラクタの中でInitialize関数を呼ぶかは要検討。
  //       今のところポートを開けたいタイミングでInitializeを読んでもらう設計思想。
  // int ret = Initialize();
}

ComPortInterface::~ComPortInterface() {
  // gcnewで確保したメモリは明示的にdeleteしなくてよい。
}

// COMポート番号（4）からCOMポート名（"COM4"）へ変換する静的メソッド
std::string ComPortInterface::PortName(int port_id) { return "COM" + std::to_string(port_id); }

int ComPortInterface::Initialize() {
  try {
    // ポートを初期設定
    port_ = gcnew IO::Ports::SerialPort(msclr::interop::marshal_as<String ^>(kPortName), kBaudRate);
  } catch (IO::IOException ^ e) {
    // ポート初期設定失敗
    return -1;
  }

  // ポートを開く。
  try {
    OpenPort();
    // エラーの詳細は以下を参照：
    // https://docs.microsoft.com/en-us/dotnet/api/system.io.ports.serialport.open?view=netframework-4.7.2
  } catch (UnauthorizedAccessException ^ e) {
    // Access is denied to the port.
    //   or
    // The current process, or another process on the system, already has the
    // specified COM port open either by a SerialPort instance or in unmanaged
    // code.
    return -2;
  } catch (ArgumentOutOfRangeException ^ e) {
    // One or more of the properties for this instance are invalid.
    return -3;
  } catch (ArgumentException ^ e) {
    // The port name does not begin with "COM".
    //   or
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
  Marshal::Copy((IntPtr)(buffer + offset), tx_buf_, 0,
                count);  // 一旦マネージドメモリへコピー
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
    Marshal::Copy(rx_buf_, 0, (IntPtr)(buffer + offset),
                  count);  // マネージドメモリから引数のメモリへコピー
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
    // ポートが開いていなかった
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
