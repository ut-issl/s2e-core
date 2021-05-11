#pragma once
#include <map>

class I2CPort
{
public:
  I2CPort(void);
	I2CPort(const unsigned char max_register_number);

  void RegisterDevice(const unsigned char i2c_addr);

  int WriteRegister(const unsigned char i2c_addr, const unsigned char reg_addr);
  int WriteRegister(const unsigned char i2c_addr, const unsigned char reg_addr, const unsigned char value);
  //int WriteRegister(const unsigned char i2c_addr, const unsigned char reg_addr, float value);  // TODO 動作確認
  
  unsigned char ReadRegister(const unsigned char i2c_addr);
  unsigned char ReadRegister(const unsigned char i2c_addr, const unsigned char reg_addr);
  
private:
  unsigned char max_register_number_ = 0xff;
  unsigned char saved_reg_addr_ = 0x00;
  // <pair(i2c_address, register address), value>
  std::map< std::pair<unsigned char, unsigned char>, unsigned char > device_registers_;
};

