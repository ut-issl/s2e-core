#pragma once
#include "../../Interface/HilsInOut/HilsPortManager.h"
#include "ObcCommunicationBase.h"

// This class simulates the I2C Controller communication with the I2C Target.
// The main purpose is to validate the emulated I2C Target component in the HILS
// test.

class I2cControllerCommunicationBase {
 public:
  I2cControllerCommunicationBase(const unsigned int hils_port_id, const unsigned int baud_rate, const unsigned int tx_buf_size,
                                 const unsigned int rx_buf_size, HilsPortManager* hils_port_manager);
  ~I2cControllerCommunicationBase();

 protected:
  int ReceiveTelemetry(const unsigned char len);
  int SendCommand(const unsigned char len);
  std::vector<unsigned char> tx_buffer_;
  std::vector<unsigned char> rx_buffer_;

 private:
  unsigned int hils_port_id_;
  int baud_rate_;  // [baud] ex. 9600, 115200
  unsigned int tx_buf_size_;
  unsigned int rx_buf_size_;
  OBC_COM_UART_MODE sim_mode_ = OBC_COM_UART_MODE::MODE_ERROR;

  HilsPortManager* hils_port_manager_;
};
