#pragma once
#ifdef USE_HILS
  #include "Ports/HilsUartPort.h"
  #include "Ports/HilsI2cTargetPort.h"
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
  virtual int UartSend(unsigned int port_id, const unsigned char* buffer, int offset, int count);

  // I2C Target Communication port functions
  virtual int I2cTargetConnectComPort(unsigned int port_id);
  virtual int I2cTargetCloseComPort(unsigned int port_id);
  virtual int I2cTargetReadRegister(unsigned int port_id, const unsigned char reg_addr,
                                    unsigned char* data, const unsigned char len);
  virtual int I2cTargetWriteRegister(unsigned int port_id, const unsigned char reg_addr,
                                     const unsigned char* data, const unsigned char len);
  virtual int I2cTargetReadCommand(unsigned int port_id, unsigned char* data, const unsigned char len);
  // I2c Com ports -> Components
  virtual int I2cTargetReceive(unsigned int port_id);
  // I2c Components -> Com ports
  virtual int I2cTargetSend(unsigned int port_id, const unsigned char len);
  virtual int I2cTargetGetStoredFrameCounter(unsigned int port_id);

  // I2C Controller Communication port functions
  virtual int I2cControllerConnectComPort(unsigned int port_id, unsigned int baud_rate, unsigned int tx_buf_size, unsigned int rx_buf_size);
  virtual int I2cControllerCloseComPort(unsigned int port_id);
  virtual int I2cControllerReceive(unsigned int port_id, unsigned char* buffer, int offset, int count);
  virtual int I2cControllerSend(unsigned int port_id, const unsigned char* buffer, int offset, int count);

private:
#ifdef USE_HILS
  // Uart ports
  std::map<int, HilsUartPort*> uart_com_ports_;
  // I2C ports
  std::map<int, HilsI2cTargetPort*> i2c_com_ports_;
#endif
};
