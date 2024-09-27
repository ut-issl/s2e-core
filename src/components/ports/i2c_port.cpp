/**
 * @file i2c_port.cpp
 * @brief Class to emulate I2C(Inter-Integrated Circuit) communication port
 */

#include "i2c_port.hpp"

#include <utilities/macros.hpp>

namespace s2e::components {

I2cPort::I2cPort(void) {}

I2cPort::I2cPort(const unsigned char max_register_number) : max_register_number_(max_register_number) {}

void I2cPort::RegisterDevice(const unsigned char i2c_address) {
  for (unsigned char i = 0; i < max_register_number_; i++) {
    device_registers_[std::make_pair(i2c_address, i)] = 0x00;
  }
  for (unsigned char i = 0; i < kDefaultCmdBufferSize; i++) {
    command_buffer_[std::make_pair(i2c_address, i)] = 0x00;
  }
}

int I2cPort::WriteRegister(const unsigned char i2c_address, const unsigned char register_address) {
  UNUSED(i2c_address);  // TODO: consider this argument is really needed.

  if (register_address >= max_register_number_) return 0;
  saved_register_address_ = register_address;
  return 1;
}

int I2cPort::WriteRegister(const unsigned char i2c_address, const unsigned char register_address, const unsigned char value) {
  if (register_address >= max_register_number_) return 0;
  saved_register_address_ = register_address;
  device_registers_[std::make_pair(i2c_address, register_address)] = value;
  return 1;
}

/*
int I2cPort::WriteRegister(const unsigned char i2c_address, const unsigned char
register_address, float value)
{
  if(register_address >= max_register_number_) return 0;
  saved_register_address_ = register_address;
  unsigned char* value_ptr = reinterpret_cast<unsigned char*>(&value);
  for(size_t i = 0; i < sizeof(float); i++)
  {
    WriteRegister(i2c_address,register_address, value_ptr[i]);
  }
  return 1;
}
*/

unsigned char I2cPort::ReadRegister(const unsigned char i2c_address) {
  unsigned char ret = device_registers_[std::make_pair(i2c_address, saved_register_address_)];
  saved_register_address_++;
  if (saved_register_address_ >= max_register_number_) saved_register_address_ = 0;
  return ret;
}

unsigned char I2cPort::ReadRegister(const unsigned char i2c_address, const unsigned char register_address) {
  if (register_address >= max_register_number_) return 0;
  saved_register_address_ = register_address;
  unsigned char ret = device_registers_[std::make_pair(i2c_address, saved_register_address_)];
  return ret;
}

unsigned char I2cPort::WriteCommand(const unsigned char i2c_address, const unsigned char* tx_data, const unsigned char length) {
  if (length > kDefaultCmdBufferSize) {
    return 0;
  }
  for (unsigned char i = 0; i < length; i++) {
    command_buffer_[std::make_pair(i2c_address, i)] = tx_data[i];
  }

  if (length == 1)  // length == 1 means setting of read register address
  {
    WriteRegister(i2c_address, tx_data[0]);
  }

  if (length == 2)  // length ==2 means setting specific register. FIXME: this rule is not general.
  {
    WriteRegister(i2c_address, tx_data[0], tx_data[1]);
  }
  return length;
}

unsigned char I2cPort::ReadCommand(const unsigned char i2c_address, unsigned char* rx_data, const unsigned char length) {
  if (length > kDefaultCmdBufferSize) {
    return 0;
  }
  for (unsigned char i = 0; i < length; i++) {
    rx_data[i] = command_buffer_[std::make_pair(i2c_address, i)];
  }
  return length;
}

} // namespace s2e::components