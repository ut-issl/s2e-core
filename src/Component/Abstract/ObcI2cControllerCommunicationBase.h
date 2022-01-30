#pragma once
#include "../CDH/OBC.h"
#include "../../Interface/HilsInOut/HilsPortManager.h"
#include "ObcCommunicationBase.h"

class ObcI2cControllerCommunicationBase
{
public:
  ObcI2cControllerCommunicationBase(
    const unsigned int hils_port_id,
    const unsigned int baud_rate,
    const int tx_buf_size,
    const int rx_buf_size,
    HilsPortManager* hils_port_manager
  );
  ~ObcI2cControllerCommunicationBase();

protected:
  int ReceiveTelemetry(const unsigned char len);
  int SendCommand(const unsigned char len);
  std::vector<unsigned char> tx_buffer_;
  std::vector<unsigned char> rx_buffer_;

private:
  int sils_port_id_;
  int hils_port_id_;
  int baud_rate_; // [baud] ex. 9600, 115200
  int tx_buf_size_;
  int rx_buf_size_;
  // unsigned char i2c_address_;
  OBC_COM_UART_MODE sim_mode_ = OBC_COM_UART_MODE::MODE_ERROR;

  HilsPortManager* hils_port_manager_;
};
