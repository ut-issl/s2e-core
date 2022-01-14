#include "HilsI2cPort.h"

HilsI2cPort::HilsI2cPort(const unsigned int port_id)
  : tx_size_(kDefaultTxSize),
    HilsUartPort(port_id, 9600, max_register_number_, kDefaultCmdSize) // TODO: Define the magic number.
{
  // TODO
  // 接続するI2C-USB変換器に応じてヘッダー・フッターを初期化
  // comポートに送信するデータをラップする
}

HilsI2cPort::HilsI2cPort(const unsigned int port_id, const unsigned char max_register_number)
  : max_register_number_(max_register_number), tx_size_(kDefaultTxSize),
    HilsUartPort(port_id, 9600, max_register_number_, kDefaultCmdSize) // TODO: Define the magic number.
{
}

HilsI2cPort::~HilsI2cPort()
{
}

void HilsI2cPort::RegisterDevice()
{
  for (unsigned char i = 0; i < max_register_number_; i++)
  {
    device_registers_[i] = 0x00;
  }
  for (unsigned char i = 0; i < kDefaultCmdSize; i++)
  {
    cmd_buffer_[i] = 0x00;
  }
}

int HilsI2cPort::WriteRegister(const unsigned char reg_addr)
{
  if (reg_addr >= max_register_number_) return 0;
  saved_reg_addr_ = reg_addr;
  return 0;
}

int HilsI2cPort::WriteRegister(const unsigned char reg_addr, const unsigned char value)
{
  if (reg_addr >= max_register_number_) return 0;
  saved_reg_addr_ = reg_addr;
  device_registers_[reg_addr] = value;
  return 0;
}

unsigned char HilsI2cPort::ReadRegister(const unsigned char reg_addr)
{
  if (reg_addr >= max_register_number_) return 0;
  saved_reg_addr_ = reg_addr;
  unsigned char ret = device_registers_[saved_reg_addr_];
  return ret;
}

int HilsI2cPort::ReadCommand(unsigned char* rx_data, const unsigned int length)
{
  if (length > kDefaultCmdSize)
  {
    return -1;
  }
  for (int i = 0; i < length; i++)
  {
    rx_data[i] = cmd_buffer_[i];
  }
  return length;
}

int HilsI2cPort::UpdateCmd()
{
  unsigned char rx_buf[kDefaultCmdSize];
  // I2C-USB変換器(Target)のbufferを定期的・高速に読み込む
  int received_bytes = ReadRx(rx_buf, 0, kDefaultCmdSize); //  TODO: Fix count size. larger than cmd size.

  if (received_bytes < 0)
  {
    return -1; // No bytes were available to read.
  }
  else if (received_bytes > kDefaultCmdSize)
  {
    return -1;
  }
  // コマンドをcmd_buffer_に格納
  for (int i = 0; i < received_bytes; i++)
  {
    cmd_buffer_[i] = rx_buf[i];
  }

  // コマンドに応じてWriteRegister()を実行
  if (received_bytes == 1) // length == 1 means setting of read register address
  {
    WriteRegister(rx_buf[0]);
  }
  if (received_bytes == 2) // length ==2 means setting specific register. FIXME: this rule is not general.
  {
    WriteRegister(rx_buf[0], rx_buf[1]);
  }
  return received_bytes;
}

int HilsI2cPort::UpdateTlm()
{
  unsigned char tx_buf[kDefaultTxSize];
  unsigned int max_tlm_size = max_register_number_ - saved_reg_addr_ + 1;
  if (tx_size_ > max_tlm_size)
  {
    tx_size_ = max_tlm_size;
  }
  for (int i = 0; i < tx_size_; i++)
  {
    tx_buf[i] = device_registers_[saved_reg_addr_ + i];
  }
  // I2C-USB変換器(Target)のbufferに定期的・高速に書き込む
  // bufferをクリア
  DiscardInBuffer();
  // テレメ送信
  int ret = WriteTx(tx_buf, 0, tx_size_);
  tx_size_ = kDefaultTxSize;
  return 0;
}
