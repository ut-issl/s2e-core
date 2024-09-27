/**
 * @file hils_port_manager.cpp
 * @brief Class to manage COM ports for HILS test
 */

#include "hils_port_manager.hpp"

#include <utilities/macros.hpp>

namespace s2e::simulation {

// #define HILS_PORT_MANAGER_SHOW_DEBUG_DATA

HilsPortManager::HilsPortManager() {}

HilsPortManager::~HilsPortManager() {}

// UART Communication port functions
int HilsPortManager::UartConnectComPort(unsigned int port_id, unsigned int baud_rate, unsigned int tx_buffer_size, unsigned int rx_buffer_size) {
#ifdef USE_HILS
  if (uart_ports_[port_id] != nullptr) {
    printf("Error: Port is already used\n");
    return -1;
  }
  if (baud_rate <= 0 || tx_buffer_size <= 0 || rx_buffer_size <= 0) {
    printf("Error: Illegal parameter\n");
    return -1;
  }
  uart_ports_[port_id] = new HilsUartPort(port_id, baud_rate, tx_buffer_size, rx_buffer_size);
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
  if (uart_ports_[port_id] == nullptr) {
    // Port not used
    return -1;
  }

  uart_ports_[port_id]->ClosePort();
  HilsUartPort* port = uart_ports_.at(port_id);
  delete port;
  uart_ports_.erase(port_id);
  return 0;
#else
  UNUSED(port_id);

  return -1;
#endif
}

int HilsPortManager::UartReceive(unsigned int port_id, unsigned char* buffer, int offset, int length) {
#ifdef USE_HILS
  HilsUartPort* port = uart_ports_[port_id];
  if (port == nullptr) return -1;
  int ret = port->ReadRx(buffer, offset, length);
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
  UNUSED(length);

  return -1;
#endif
}

int HilsPortManager::UartSend(unsigned int port_id, const unsigned char* buffer, int offset, int length) {
#ifdef USE_HILS
  HilsUartPort* port = uart_ports_[port_id];
  if (port == nullptr) return -1;
  int ret = port->WriteTx(buffer, offset, length);
#ifdef HILS_PORT_MANAGER_SHOW_DEBUG_DATA
  if (length > 0) {
    printf("UART PORT ID: %d sent %d bytes\n", port_id, length);
    for (int i = 0; i < length; i++) {
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
  UNUSED(length);

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

int HilsPortManager::I2cTargetWriteRegister(unsigned int port_id, const unsigned char register_address, const unsigned char* data,
                                            const unsigned char length) {
#ifdef USE_HILS
  HilsI2cTargetPort* port = i2c_ports_[port_id];
  if (port == nullptr) return -1;
  for (unsigned char i = 0; i < length; i++) {
    port->WriteRegister(register_address + i, data[i]);
  }
  return 0;
#else
  UNUSED(port_id);
  UNUSED(register_address);
  UNUSED(data);
  UNUSED(length);

  return -1;
#endif
}

int HilsPortManager::I2cTargetReadRegister(unsigned int port_id, const unsigned char register_address, unsigned char* data,
                                           const unsigned char length) {
#ifdef USE_HILS
  HilsI2cTargetPort* port = i2c_ports_[port_id];
  if (port == nullptr) return -1;
  for (unsigned char i = 0; i < length; i++) {
    data[i] = port->ReadRegister(register_address + i);
  }
  return 0;
#else
  UNUSED(port_id);
  UNUSED(register_address);
  UNUSED(data);
  UNUSED(length);

  return -1;
#endif
}

int HilsPortManager::I2cTargetReadCommand(unsigned int port_id, unsigned char* data, const unsigned char length) {
#ifdef USE_HILS
  HilsI2cTargetPort* port = i2c_ports_[port_id];
  if (port == nullptr) return -1;
  port->ReadCommand(data, length);
  return 0;
#else
  UNUSED(port_id);
  UNUSED(data);
  UNUSED(length);

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

int HilsPortManager::I2cTargetSend(unsigned int port_id, const unsigned char length) {
#ifdef USE_HILS
  HilsI2cTargetPort* port = i2c_ports_[port_id];
  if (port == nullptr) return -1;
  int ret = port->Send(length);
#ifdef HILS_PORT_MANAGER_SHOW_DEBUG_DATA
  if (length > 0) {
    printf("I2C PORT ID: %d sent %d bytes\n", port_id, length);
  }
#endif
  return ret;
#else
  UNUSED(port_id);
  UNUSED(length);

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

int HilsPortManager::I2cControllerReceive(unsigned int port_id, unsigned char* buffer, int offset, int length) {
  return UartReceive(port_id, buffer, offset, length);
}

int HilsPortManager::I2cControllerSend(unsigned int port_id, const unsigned char* buffer, int offset, int length) {
  return UartSend(port_id, buffer, offset, length);
}

} // namespace s2e::simulation
