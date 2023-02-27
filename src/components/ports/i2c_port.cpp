/**
 * @file i2c_port.cpp
 * @brief Class to emulate I2C(Inter-Integrated Circuit) communication port
 */

#include "i2c_port.hpp"

#include <library/utilities/macros.hpp>

I2CPort::I2CPort(void) {}

I2CPort::I2CPort(const unsigned char max_register_number) : max_register_number_(max_register_number) {}

void I2CPort::RegisterDevice(const unsigned char i2c_addr) {
  for (unsigned char i = 0; i < max_register_number_; i++) {
    device_registers_[std::make_pair(i2c_addr, i)] = 0x00;
  }
  for (unsigned char i = 0; i < kDefaultCmdBufferSize; i++) {
    command_buffer_[std::make_pair(i2c_addr, i)] = 0x00;
  }
}

int I2CPort::WriteRegister(const unsigned char i2c_addr, const unsigned char reg_addr) {
  UNUSED(i2c_addr);  // TODO: consider this argument is really needed.

  if (reg_addr >= max_register_number_) return 0;
  saved_register_address_ = reg_addr;
  return 1;
}

int I2CPort::WriteRegister(const unsigned char i2c_addr, const unsigned char reg_addr, const unsigned char value) {
  if (reg_addr >= max_register_number_) return 0;
  saved_register_address_ = reg_addr;
  device_registers_[std::make_pair(i2c_addr, reg_addr)] = value;
  return 1;
}

/*
int I2CPort::WriteRegister(const unsigned char i2c_addr, const unsigned char
reg_addr, float value)
{
  if(reg_addr >= max_register_number_) return 0;
  saved_register_address_ = reg_addr;
  unsigned char* value_ptr = reinterpret_cast<unsigned char*>(&value);
  for(size_t i = 0; i < sizeof(float); i++)
  {
    WriteRegister(i2c_addr,reg_addr, value_ptr[i]);
  }
  return 1;
}
*/

unsigned char I2CPort::ReadRegister(const unsigned char i2c_addr) {
  unsigned char ret = device_registers_[std::make_pair(i2c_addr, saved_register_address_)];
  saved_register_address_++;
  if (saved_register_address_ >= max_register_number_) saved_register_address_ = 0;
  return ret;
}

unsigned char I2CPort::ReadRegister(const unsigned char i2c_addr, const unsigned char reg_addr) {
  if (reg_addr >= max_register_number_) return 0;
  saved_register_address_ = reg_addr;
  unsigned char ret = device_registers_[std::make_pair(i2c_addr, saved_register_address_)];
  return ret;
}

unsigned char I2CPort::WriteCommand(const unsigned char i2c_addr, const unsigned char* tx_data, const unsigned char length) {
  if (length > kDefaultCmdBufferSize) {
    return 0;
  }
  for (unsigned char i = 0; i < length; i++) {
    command_buffer_[std::make_pair(i2c_addr, i)] = tx_data[i];
  }

  if (length == 1)  // length == 1 means setting of read register address
  {
    WriteRegister(i2c_addr, tx_data[0]);
  }

  if (length == 2)  // length ==2 means setting specific register. FIXME: this rule is not general.
  {
    WriteRegister(i2c_addr, tx_data[0], tx_data[1]);
  }
  return length;
}

unsigned char I2CPort::ReadCommand(const unsigned char i2c_addr, unsigned char* rx_data, const unsigned char length) {
  if (length > kDefaultCmdBufferSize) {
    return 0;
  }
  for (unsigned char i = 0; i < length; i++) {
    rx_data[i] = command_buffer_[std::make_pair(i2c_addr, i)];
  }
  return length;
}
