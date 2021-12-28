#include "HilsPortManager.h"

HilsPortManager::HilsPortManager()
{
}

HilsPortManager::~HilsPortManager()
{
}

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

int HilsPortManager::UartSend(unsigned int port_id, unsigned char* buffer, int offset, int count)
{
#ifdef USE_HILS
  HilsUartPort* port = uart_com_ports_[port_id];
  if (port == nullptr) return -1;
  return port->WriteTx(buffer, offset, count);
#else
  return -1;
#endif
}
