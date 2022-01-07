#pragma once
#include "../CDH/OBC.h"
#include "../../Interface/HilsInOut/HilsPortManager.h"
#include "ObcCommunicationBase.h"

class ObcI2cCommunicationBase
{
public:
  ObcI2cCommunicationBase(
    const int sils_port_id,
    const unsigned char i2c_address,
    OBC* obc
  );
  ObcI2cCommunicationBase(
    const unsigned int hils_port_id,
    HilsPortManager* hils_port_manager
  );
  ObcI2cCommunicationBase(
    const int sils_port_id,
    const unsigned char i2c_address,
    OBC* obc,
    const unsigned int hils_port_id,
    HilsPortManager* hils_port_manager
  );
  ~ObcI2cCommunicationBase();

protected:
  void ReadRegister (const unsigned char reg_addr, unsigned char* data, const unsigned char len);
  void WriteRegister(const unsigned char reg_addr, const unsigned char* data, const unsigned char len); 
  void ReadCommand  (unsigned char* data, const unsigned char len);
  
private:
  const int kDefaultBufferSize = 512;  // Fixme: The magic number. This is depending on USB-I2C converter.
  const int kDefaultBaudRate   = 9600; // Any value is fine for I2C slave.
  int sils_port_id_;
  int hils_port_id_;
  unsigned char i2c_address_;
  int baud_rate_; // [baud] ex. 9600, 115200
  int tx_buf_size_;
  int rx_buf_size_;
  OBC_COM_UART_MODE sim_mode_ = OBC_COM_UART_MODE::MODE_ERROR;

  OBC* obc_;
  HilsPortManager* hils_port_manager_;
};