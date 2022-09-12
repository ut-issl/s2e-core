#include "HilsI2cTargetPort.h"

// #define HILS_I2C_TARGET_PORT_SHOW_DEBUG_DATA

HilsI2cTargetPort::HilsI2cTargetPort(const unsigned int port_id)
    : HilsUartPort(port_id, 115200, 512,
                   512)  // Fixme: The magic number. This is depending on the converter.
{}

HilsI2cTargetPort::HilsI2cTargetPort(const unsigned int port_id, const unsigned char max_register_number)
    : max_register_number_(max_register_number),
      HilsUartPort(port_id, 115200, 512,
                   512)  // Fixme: The magic number. This is depending on the converter.
{}

HilsI2cTargetPort::~HilsI2cTargetPort() {}

void HilsI2cTargetPort::RegisterDevice() {
  for (unsigned char i = 0; i < max_register_number_; i++) {
    device_registers_[i] = 0x00;
  }
  for (unsigned char i = 0; i < kDefaultCmdSize; i++) {
    cmd_buffer_[i] = 0x00;
  }
}

int HilsI2cTargetPort::WriteRegister(const unsigned char reg_addr) {
  if (reg_addr >= max_register_number_) return 0;
  saved_reg_addr_ = reg_addr;
  return 0;
}

int HilsI2cTargetPort::WriteRegister(const unsigned char reg_addr, const unsigned char value) {
  if (reg_addr >= max_register_number_) return 0;
  saved_reg_addr_ = reg_addr;
  device_registers_[reg_addr] = value;
  return 0;
}

unsigned char HilsI2cTargetPort::ReadRegister(const unsigned char reg_addr) {
  if (reg_addr >= max_register_number_) return 0;
  saved_reg_addr_ = reg_addr;
  unsigned char ret = device_registers_[reg_addr];
  return ret;
}

int HilsI2cTargetPort::ReadCommand(unsigned char* rx_data, const unsigned int length) {
  if (length > kDefaultCmdSize) {
    return -1;
  }
  for (unsigned char i = 0; i < length; i++) {
    rx_data[i] = cmd_buffer_[i];
  }
  return length;
}

int HilsI2cTargetPort::Receive()  // from I2C-USB Target converter
{
  unsigned char rx_buf[kDefaultCmdSize];
  if (GetBytesToRead() <= 0) return -1;  // No bytes were available to read.
  int received_bytes = ReadRx(rx_buf, 0, kDefaultCmdSize);
  if (received_bytes > kDefaultCmdSize) return -1;
#ifdef HILS_I2C_TARGET_PORT_SHOW_DEBUG_DATA
  for (int i = 0; i < received_bytes; i++)
  {
    printf("%02x ", rx_buf[i]);
  }
  printf("\n");
#endif
  for (unsigned char i = 0; i < received_bytes; i++) {
    cmd_buffer_[i] = rx_buf[i];
  }

  if (received_bytes == 1)  // length == 1 means setting of read register address
  {
    WriteRegister(rx_buf[0]);
    if (stored_frame_counter_ > 0) {
      stored_frame_counter_ = stored_frame_counter_ - 1;
    }
  }
  if (received_bytes == 2)  // length == 2 means setting specific register.
                            // FIXME: this rule is not general.
  {
    WriteRegister(rx_buf[0], rx_buf[1]);
  }
  return received_bytes;
}

int HilsI2cTargetPort::Send(const unsigned char len)  // to I2C-USB Target Converter
{
  if (saved_reg_addr_ + len > max_register_number_) return -1;
  unsigned char tx_buf[kDefaultTxSize] = {0};
  for (unsigned char i = 0; i < len; i++) {
    tx_buf[i] = device_registers_[saved_reg_addr_ + i];
  }
#ifdef HILS_I2C_TARGET_PORT_SHOW_DEBUG_DATA
  for (int i = 0; i < len; i++)
  {
    printf("%02x ", tx_buf[i]);
  }
  printf("\n");
#endif
  int ret = WriteTx(tx_buf, 0, len);
  stored_frame_counter_++;
  return ret;
}

int HilsI2cTargetPort::GetStoredFrameCounter() { return stored_frame_counter_; }
