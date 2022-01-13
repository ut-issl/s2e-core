#include "HilsI2cPort.h"

HilsI2cPort::HilsI2cPort(const unsigned int port_id)
  : HilsUartPort(port_id, 9600, 512, 512) // TODO: Define the magic number.
{
  // TODO
  // 接続するI2C-USB変換器に応じてヘッダー・フッターを初期化
  // comポートに送信するデータをラップする
}

HilsI2cPort::HilsI2cPort(const unsigned int port_id, const unsigned char max_register_number)
  : max_register_number_(max_register_number), HilsUartPort(port_id, 9600, 512, 512) // TODO: Define the magic number.
{
}

HilsI2cPort::~HilsI2cPort()
{
}

void HilsI2cPort::RegisterDevice()
{

}

int HilsI2cPort::WriteRegister(const unsigned char reg_addr, const unsigned char value)
{
  return 0;
}

unsigned char HilsI2cPort::ReadRegister(const unsigned char reg_addr)
{
  return 0;
}

int HilsI2cPort::ReadCommand(unsigned char* rx_data, const unsigned int length)
{
  return 0;
}

int HilsI2cPort::UpdateCmd()
{
  // I2C-USB変換器(Target)のbufferを定期的・高速に読み込む: HilsUartPort::ReadRx()
  // コマンドをcmd_buffer_に格納
  // コマンドに応じてWriteRegister()を実行

  return 0;
}

int HilsI2cPort::UpdateTlm()
{
  // I2C-USB変換器(Target)のbufferに定期的・高速に書き込む: HilsUartPort::WriteTx()
  // まずbufferをクリア
  // saved_reg_addrから十分なbyte数分のデータをdevice_register_から送る

  return 0;
}
