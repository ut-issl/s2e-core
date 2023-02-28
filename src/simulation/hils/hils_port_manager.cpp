/**
 * @file hils_port_manager.cpp
 * @brief Class to manage COM ports for HILS test
 */

#include "hils_port_manager.hpp"

#include <library/utilities/macros.hpp>

// #define HILS_PORT_MANAGER_SHOW_DEBUG_DATA

HilsPortManager::HilsPortManager() {}

HilsPortManager::~HilsPortManager() {}

// UART Communication port functions
int HilsPortManager::UartConnectComPort(unsigned int port_id, unsigned int baud_rate, unsigned int tx_buffer_size, unsigned int rx_buffer_size) {
#ifdef USE_HILS
  if (uart_com_ports_[port_id] != nullptr) {
    printf("Error: Port is already used\n");
    return -1;
  }
  if (baud_rate <= 0 || tx_buffer_size <= 0 || rx_buffer_size <= 0) {
    printf("Error: Illegal parameter\n");
    return -1;
  }
  uart_com_ports_[port_id] = new HilsUartPort(port_id, baud_rate, tx_buffer_size, rx_buffer_size);
  return 0;
#else
  UNUSED(port_id);
  UNUSED(baud_rate);
  UNUSED(tx_buffer_size);
  UNUSED(rx_buffer_size);

  return -1;
#endif
}

// Close port and free resources
int HilsPortManager::UartCloseComPort(unsigned int port_id) {
#ifdef USE_HILS
  if (uart_com_ports_[port_id] == nullptr) {
    // Port not used
    return -1;
  }

  uart_com_ports_[port_id]->ClosePort();
  HilsUartPort* port = uart_com_ports_.at(port_id);
  delete port;
  uart_com_ports_.erase(port_id);
  return 0;
#else
  UNUSED(port_id);

  return -1;
#endif
}

int HilsPortManager::UartReceive(unsigned int port_id, unsigned char* buffer, int offset, int count) {
#ifdef USE_HILS
  HilsUartPort* port = uart_com_ports_[port_id];
  if (port == nullptr) return -1;
  int ret = port->ReadRx(buffer, offset, count);
#ifdef HILS_PORT_MANAGER_SHOW_DEBUG_DATA
  if (ret > 0) {
    printf("UART PORT ID: %d received %d bytes\n", port_id, ret);
    for (int i = 0; i < ret; i++) {
      printf("%02x ", buffer[i]);
    }
    printf("\n");
  }
#endif
  return ret;
#else
  UNUSED(port_id);
  UNUSED(buffer);
  UNUSED(offset);
  UNUSED(count);

  return -1;
#endif
}

int HilsPortManager::UartSend(unsigned int port_id, const unsigned char* buffer, int offset, int count) {
#ifdef USE_HILS
  HilsUartPort* port = uart_com_ports_[port_id];
  if (port == nullptr) return -1;
  int ret = port->WriteTx(buffer, offset, count);
#ifdef HILS_PORT_MANAGER_SHOW_DEBUG_DATA
  if (count > 0) {
    printf("UART PORT ID: %d sent %d bytes\n", port_id, count);
    for (int i = 0; i < count; i++) {
      printf("%02x ", buffer[i]);
    }
    printf("\n");
  }
#endif
  return ret;
#else
  UNUSED(port_id);
  UNUSED(buffer);
  UNUSED(offset);
  UNUSED(count);

  return -1;
#endif
}

// I2C Target Communication port functions
int HilsPortManager::I2cTargetConnectComPort(unsigned int port_id) {
#ifdef USE_HILS
  if (i2c_ports_[port_id] != nullptr) {
    printf("Error: Port is already used\n");
    return -1;
  }
  i2c_ports_[port_id] = new HilsI2cTargetPort(port_id);
  i2c_ports_[port_id]->RegisterDevice();
  return 0;
#else
  UNUSED(port_id);

  return -1;
#endif
}

int HilsPortManager::I2cTargetCloseComPort(unsigned int port_id) {
#ifdef USE_HILS
  if (i2c_ports_[port_id] == nullptr) {
    // Port not used
    return -1;
  }
  i2c_ports_[port_id]->ClosePort();
  HilsI2cTargetPort* port = i2c_ports_.at(port_id);
  delete port;
  i2c_ports_.erase(port_id);
  return 0;
#else
  UNUSED(port_id);

  return -1;
#endif
}

int HilsPortManager::I2cTargetWriteRegister(unsigned int port_id, const unsigned char reg_addr, const unsigned char* data, const unsigned char len) {
#ifdef USE_HILS
  HilsI2cTargetPort* port = i2c_ports_[port_id];
  if (port == nullptr) return -1;
  for (unsigned char i = 0; i < len; i++) {
    port->WriteRegister(reg_addr + i, data[i]);
  }
  return 0;
#else
  UNUSED(port_id);
  UNUSED(reg_addr);
  UNUSED(data);
  UNUSED(len);

  return -1;
#endif
}

int HilsPortManager::I2cTargetReadRegister(unsigned int port_id, const unsigned char reg_addr, unsigned char* data, const unsigned char len) {
#ifdef USE_HILS
  HilsI2cTargetPort* port = i2c_ports_[port_id];
  if (port == nullptr) return -1;
  for (unsigned char i = 0; i < len; i++) {
    data[i] = port->ReadRegister(reg_addr + i);
  }
  return 0;
#else
  UNUSED(port_id);
  UNUSED(reg_addr);
  UNUSED(data);
  UNUSED(len);

  return -1;
#endif
}

int HilsPortManager::I2cTargetReadCommand(unsigned int port_id, unsigned char* data, const unsigned char len) {
#ifdef USE_HILS
  HilsI2cTargetPort* port = i2c_ports_[port_id];
  if (port == nullptr) return -1;
  port->ReadCommand(data, len);
  return 0;
#else
  UNUSED(port_id);
  UNUSED(data);
  UNUSED(len);

  return -1;
#endif
}

int HilsPortManager::I2cTargetReceive(unsigned int port_id) {
#ifdef USE_HILS
  HilsI2cTargetPort* port = i2c_ports_[port_id];
  if (port == nullptr) return -1;
  int ret = port->Receive();
#ifdef HILS_PORT_MANAGER_SHOW_DEBUG_DATA
  if (ret > 0) {
    printf("I2C PORT ID: %d received %d bytes\n", port_id, ret);
  }
#endif
  return ret;
#else
  UNUSED(port_id);

  return -1;
#endif
}

int HilsPortManager::I2cTargetSend(unsigned int port_id, const unsigned char len) {
#ifdef USE_HILS
  HilsI2cTargetPort* port = i2c_ports_[port_id];
  if (port == nullptr) return -1;
  int ret = port->Send(len);
#ifdef HILS_PORT_MANAGER_SHOW_DEBUG_DATA
  if (len > 0) {
    printf("I2C PORT ID: %d sent %d bytes\n", port_id, len);
  }
#endif
  return ret;
#else
  UNUSED(port_id);
  UNUSED(len);

  return -1;
#endif
}

int HilsPortManager::I2cTargetGetStoredFrameCounter(unsigned int port_id) {
#ifdef USE_HILS
  HilsI2cTargetPort* port = i2c_ports_[port_id];
  if (port == nullptr) return -1;
  return port->GetStoredFrameCounter();
#else
  UNUSED(port_id);

  return -1;
#endif
}

// I2C Controller Communication port functions
int HilsPortManager::I2cControllerConnectComPort(unsigned int port_id, unsigned int baud_rate, unsigned int tx_buffer_size,
                                                 unsigned int rx_buffer_size) {
  return UartConnectComPort(port_id, baud_rate, tx_buffer_size, rx_buffer_size);
}

int HilsPortManager::I2cControllerCloseComPort(unsigned int port_id) { return UartCloseComPort(port_id); }

int HilsPortManager::I2cControllerReceive(unsigned int port_id, unsigned char* buffer, int offset, int count) {
  return UartReceive(port_id, buffer, offset, count);
}

int HilsPortManager::I2cControllerSend(unsigned int port_id, const unsigned char* buffer, int offset, int count) {
  return UartSend(port_id, buffer, offset, count);
}
