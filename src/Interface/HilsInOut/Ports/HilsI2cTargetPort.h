#pragma once
#include <map>

#include "HilsUartPort.h"

// I2C-USB converter is assumed to be used.
// The emulated component transmits to and receives from the converter via the
// COM port.

const int kDefaultCmdSize = 0xff;
const int kDefaultTxSize = 0xff;

// Communication between I2C-USB converter and emulated component via COM port
// is the same as UART.
class HilsI2cTargetPort : public HilsUartPort {
 public:
  HilsI2cTargetPort(const unsigned int port_id);
  HilsI2cTargetPort(const unsigned int port_id, const unsigned char max_register_number);
  ~HilsI2cTargetPort();

  void RegisterDevice();

  int WriteRegister(const unsigned char reg_addr);
  int WriteRegister(const unsigned char reg_addr, const unsigned char value);
  unsigned char ReadRegister(const unsigned char reg_addr);
  int ReadCommand(unsigned char* rx_data, const unsigned int length);

  int Receive();
  int Send(const unsigned char len);
  int GetStoredFrameCounter();

 private:
  unsigned char max_register_number_ = 0xff;
  unsigned char saved_reg_addr_ = 0x00;
  unsigned int stored_frame_counter_ = 0;  // Send a few frames of telemetry to the converter in advance.

  // < register address, value>
  std::map<unsigned char, unsigned char> device_registers_;

  // <cmd_buffer_length, value>
  std::map<unsigned char, unsigned char> cmd_buffer_;
};
