#include "HilsPortManager.h"

HilsPortManager::HilsPortManager()
{
}

HilsPortManager::~HilsPortManager()
{
}

// UART Communication port functions
int HilsPortManager::UartConnectComPort(unsigned int port_id, unsigned int baud_rate, unsigned int tx_buf_size, unsigned int rx_buf_size)
{
#ifdef USE_HILS
  if (uart_com_ports_[port_id] != nullptr)
  {
    printf("Error: Port is already used\n");
    return -1;
  }
  if (baud_rate <= 0 || tx_buf_size <= 0 || rx_buf_size <= 0)
  {
    printf("Error: Illegal parameter\n");
    return -1;
  }
  uart_com_ports_[port_id] = new HilsUartPort(port_id, baud_rate, tx_buf_size, rx_buf_size);
  return 0;
#else
  return -1;
#endif
}

// Close port and free resources
int HilsPortManager::UartCloseComPort(unsigned int port_id)
{
#ifdef USE_HILS
  if (uart_com_ports_[port_id] == nullptr)
  {
    // Port not used
    return -1;
  }

  uart_com_ports_[port_id]->ClosePort();
  HilsUartPort* port = uart_com_ports_.at(port_id);
  delete port;
  uart_com_ports_.erase(port_id);
  return 0;
#else
  return -1;
#endif
}

int HilsPortManager::UartReceive(unsigned int port_id, unsigned char* buffer, int offset, int count)
{
#ifdef USE_HILS
  HilsUartPort* port = uart_com_ports_[port_id];
  if (port == nullptr) return -1;
  return port->ReadRx(buffer, offset, count);
#else
  return -1;
#endif
}

int HilsPortManager::UartSend(unsigned int port_id, const unsigned char* buffer, int offset, int count)
{
#ifdef USE_HILS
  HilsUartPort* port = uart_com_ports_[port_id];
  if (port == nullptr) return -1;
  return port->WriteTx(buffer, offset, count);
#else
  return -1;
#endif
}

// I2C Communication port functions
int HilsPortManager::I2cConnectComPort(unsigned int port_id)
{
#ifdef USE_HILS
  if (i2c_com_ports_[port_id] != nullptr)
  {
    printf("Error: Port is already used\n");
    return -1;
  }
  i2c_com_ports_[port_id] = new HilsI2cPort(port_id);
  return 0;
#else
  return -1;
#endif
}

int HilsPortManager::I2cCloseComPort(unsigned int port_id)
{
#ifdef USE_HILS
  if (uart_com_ports_[port_id] == nullptr)
  {
    // Port not used
    return -1;
  }
  i2c_com_ports_[port_id]->ClosePort();
  HilsI2cPort* port = i2c_com_ports_.at(port_id);
  delete port;
  i2c_com_ports_.erase(port_id);
  return 0;
#else
  return -1;
#endif
}

int HilsPortManager::I2cTargetWriteRegister(unsigned int port_id, const unsigned char reg_addr, const unsigned char* data, const unsigned char len)
{
#ifdef USE_HILS
  HilsI2cPort* port = i2c_com_ports_[port_id];
  if (port == nullptr) return -1;
  for (int i = 0; i < len; i++)
  {
    port->WriteRegister(reg_addr + i, data[i]);
  }
  return 0;
#else
  return -1;
#endif
}

int HilsPortManager::I2cTargetReadRegister(unsigned int port_id, const unsigned char reg_addr, unsigned char* data, const unsigned char len)
{
#ifdef USE_HILS
  HilsI2cPort* port = i2c_com_ports_[port_id];
  if (port == nullptr) return -1;
  for (int i = 0; i < len; i++)
  {
    data[i] = port->ReadRegister(reg_addr + i);
  }
  return 0;
#else
  return -1;
#endif
}

int HilsPortManager::I2cTargetReadCommand(unsigned int port_id, unsigned char* data, const unsigned char len)
{
#ifdef USE_HILS
  HilsI2cPort* port = i2c_com_ports_[port_id];
  if (port == nullptr) return -1;
  port->ReadCommand(data, len);
  return 0;
#else
  return -1;
#endif
}

int HilsPortManager::I2cTargetUpdateCmd(unsigned int port_id)
{
  // I2C-USB変換器(Target)のbufferを定期的・高速に読み込む
#ifdef USE_HILS
  HilsI2cPort* port = i2c_com_ports_[port_id];
  if (port == nullptr) return -1;
  port->UpdateCmd();
  return 0;
#else
  return -1;
#endif
}

int HilsPortManager::I2cTargetUpdateTlm(unsigned int port_id)
{
  // I2C-USB変換器(Target)のbufferに定期的・高速に書き込む
#ifdef USE_HILS
  HilsI2cPort* port = i2c_com_ports_[port_id];
  if (port == nullptr) return -1;
  port->UpdateTlm();
  return 0;
#else
  return -1;
#endif
}
