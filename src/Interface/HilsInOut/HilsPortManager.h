#pragma once
#ifdef USE_HILS
  #include "Ports/HilsUartPort.h"
#endif
#include <map>

class HilsPortManager
{
public:
  HilsPortManager();
  virtual ~HilsPortManager();

  // Uart Communication port functions
  virtual int UartConnectComPort(unsigned int port_id,
                                 unsigned int baud_rate, // [baud] ex 9600, 115200
                                 unsigned int tx_buf_size, unsigned int rx_buf_size);
  virtual int UartCloseComPort(unsigned int port_id);
  // Uart Com ports -> Components
  virtual int UartReceive(unsigned int port_id, unsigned char* buffer, int offset, int count);
  // Uart Components -> Com ports
  virtual int UartSend(unsigned int port_id, unsigned char* buffer, int offset, int count);

  // TODO: Add I2C Communication port functions

private:
  // Uart ports
#ifdef USE_HILS
  std::map<int, HilsUartPort*> uart_com_ports_;
#endif
  // I2C ports
  // std::map<int, I2CPort*> i2c_com_ports_;
};
