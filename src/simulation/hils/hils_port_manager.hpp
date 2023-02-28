/**
 * @file hils_port_manager.hpp
 * @brief Class to manage COM ports for HILS test
 */

#ifndef S2E_SIMULATION_HILS_HILS_PORT_MANAGER_HPP_
#define S2E_SIMULATION_HILS_HILS_PORT_MANAGER_HPP_

#ifdef USE_HILS
#include "ports/hils_i2c_target_port.hpp"
#include "ports/hils_uart_port.hpp"
#endif
#include <map>

/**
 * @class HilsPortManager
 * @brief Class to manage COM ports for HILS test
 */
class HilsPortManager {
 public:
  /**
   * @fn HilsPortManager
   * @brief Constructor.
   */
  HilsPortManager();
  /**
   * @fn ~HilsPortManager
   * @brief Destructor.
   */
  virtual ~HilsPortManager();

  // UART Communication port functions
  /**
   * @fn UartConnectComPort
   * @brief Connect UART port
   * @param [in] port_id: COM port ID
   * @param [in] baud_rate: Baud rate ex 9600, 115200
   * @param [in] tx_buffer_size: TX buffer size
   * @param [in] rx_buffer_size: RX buffer size
   */
  virtual int UartConnectComPort(unsigned int port_id, unsigned int baud_rate, unsigned int tx_buffer_size, unsigned int rx_buffer_size);
  /**
   * @fn UartCloseComPort
   * @brief Close UART port
   * @param [in] port_id: COM port ID
   */
  virtual int UartCloseComPort(unsigned int port_id);
  /**
   * @fn UartReceive
   * @brief UART data receive from COM port (ex. OBC) to components in S2E
   * @param [in] port_id: COM port ID
   * @param [out] buffer: Data buffer to receive
   * @param [in] offset: Start offset for the data buffer to receive
   * @param [in] count: Length of data to receive
   */
  virtual int UartReceive(unsigned int port_id, unsigned char* buffer, int offset, int count);
  /**
   * @fn UartSend
   * @brief UART data send from components in S2E to COM port (ex. OBC)
   * @param [in] port_id: COM port ID
   * @param [in] buffer: Data buffer to send
   * @param [in] offset: Start offset for the data buffer to send
   * @param [in] count: Length of data to send
   */
  virtual int UartSend(unsigned int port_id, const unsigned char* buffer, int offset, int count);

  // I2C Target Communication port functions
  /**
   * @fn I2cTargetConnectComPort
   * @brief Connect I2C port as target device
   * @param [in] port_id: COM port ID
   */
  virtual int I2cTargetConnectComPort(unsigned int port_id);
  /**
   * @fn I2cTargetCloseComPort
   * @brief Close I2C port as target device
   * @param [in] port_id: COM port ID
   */
  virtual int I2cTargetCloseComPort(unsigned int port_id);
  /**
   * @fn I2cTargetReadRegister
   * @brief Read I2C register in S2E
   * @param [in] port_id: COM port ID
   * @param [in] reg_addr: Register address to read
   * @param [out] data: Data buffer to store the read data
   * @param [in] len: Read data length
   */
  virtual int I2cTargetReadRegister(unsigned int port_id, const unsigned char reg_addr, unsigned char* data, const unsigned char len);
  /**
   * @fn I2cTargetWriteRegister
   * @brief Write data to I2C register in S2E
   * @param [in] port_id: COM port ID
   * @param [in] reg_addr: Register address to write
   * @param [in] data: Data to write
   * @param [in] len: Write data length
   */
  virtual int I2cTargetWriteRegister(unsigned int port_id, const unsigned char reg_addr, const unsigned char* data, const unsigned char len);
  /**
   * @fn I2cTargetReadCommand
   * @brief Read I2C command buffer in S2E
   * @param [in] port_id: COM port ID
   * @param [out] data: Data buffer to store the read data
   * @param [in] len: Read data length
   */
  virtual int I2cTargetReadCommand(unsigned int port_id, unsigned char* data, const unsigned char len);

  /**
   * @fn I2cTargetReceive
   * @brief Read data from the I2C-USB converter
   * @param [in] port_id: COM port ID
   */
  virtual int I2cTargetReceive(unsigned int port_id);
  /**
   * @fn I2cTargetSend
   * @brief Send data to the I2C-USB converter
   * @param [in] port_id: COM port ID
   * @param [in] len: Data length to write
   */
  virtual int I2cTargetSend(unsigned int port_id, const unsigned char len);
  /**
   * @fn I2cTargetGetStoredFrameCounter
   * @brief Get stored frame counter
   * @param [in] port_id: COM port ID
   * @return -1: error, others: stored frame counter
   */
  virtual int I2cTargetGetStoredFrameCounter(unsigned int port_id);

  // I2C Controller Communication port functions
  /**
   * @fn I2cControllerConnectComPort
   * @brief Connect I2C controller side device on the COM port
   * @param [in] port_id: COM port ID
   * @param [in] baud_rate: Baud rate ex 9600, 115200
   * @param [in] tx_buffer_size: TX buffer size
   * @param [in] rx_buffer_size: RX buffer size
   */
  virtual int I2cControllerConnectComPort(unsigned int port_id, unsigned int baud_rate, unsigned int tx_buffer_size, unsigned int rx_buffer_size);
  /**
   * @fn I2cControllerCloseComPort
   * @brief Close I2C controller side device on the COM port
   * @param [in] port_id: COM port ID
   */
  virtual int I2cControllerCloseComPort(unsigned int port_id);
  /**
   * @fn I2cControllerReceive
   * @brief Data receive from I2C controller device connected a COM port
   * @param [in] port_id: COM port ID
   * @param [out] buffer: Data buffer to receive
   * @param [in] offset: Start offset for the data buffer to receive
   * @param [in] count: Length of data to receive
   */
  virtual int I2cControllerReceive(unsigned int port_id, unsigned char* buffer, int offset, int count);
  /**
   * @fn I2cControllerSend
   * @brief Data send to I2C controller device connected a COM port
   * @param [in] port_id: COM port ID
   * @param [in] buffer: Data buffer to send
   * @param [in] offset: Start offset for the data buffer to send
   * @param [in] count: Length of data to send
   */
  virtual int I2cControllerSend(unsigned int port_id, const unsigned char* buffer, int offset, int count);

 private:
#ifdef USE_HILS
  std::map<int, HilsUartPort*> uart_com_ports_;  //!< UART ports
  std::map<int, HilsI2cTargetPort*> i2c_ports_;  //!< I2C ports
#endif
};

#endif  // S2E_SIMULATION_HILS_HILS_PORT_MANAGER_HPP_
