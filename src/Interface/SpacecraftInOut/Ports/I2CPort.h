#pragma once
#include <map>

const int kDefaultCmdBufferSize = 0xff;

class I2CPort {
public:
  I2CPort(void);
  I2CPort(const unsigned char max_register_number);

  void RegisterDevice(const unsigned char i2c_addr);

  int WriteRegister(const unsigned char i2c_addr, const unsigned char reg_addr);
  int WriteRegister(const unsigned char i2c_addr, const unsigned char reg_addr,
                    const unsigned char value);
  // int WriteRegister(const unsigned char i2c_addr, const unsigned char
  // reg_addr, float value);  // TODO 動作確認

  unsigned char ReadRegister(const unsigned char i2c_addr);
  unsigned char ReadRegister(const unsigned char i2c_addr,
                             const unsigned char reg_addr);

  // OBC->Component Command emulation
  int WriteCommand(const unsigned char i2c_addr, const unsigned char *tx_data,
                   const unsigned int length);
  int ReadCommand(const unsigned char i2c_addr, unsigned char *rx_data,
                  const unsigned int length);

private:
  unsigned char max_register_number_ = 0xff;
  unsigned char saved_reg_addr_ = 0x00;

  // <pair(i2c_address, register address), value>
  std::map<std::pair<unsigned char, unsigned char>, unsigned char>
      device_registers_;

  // OBC->Component Command emulation i2c_address, buffer_
  // <pair(i2c_address, cmd_buffer_length), value>
  std::map<std::pair<unsigned char, unsigned char>, unsigned char> cmd_buffer_;
};
