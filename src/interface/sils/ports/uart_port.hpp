/**
 * @file uart_port.hpp
 * @brief Class to emulate UART communication port
 */

#ifndef S2E_INTERFACE_SILS_PORTS_UART_PORT_HPP_
#define S2E_INTERFACE_SILS_PORTS_UART_PORT_HPP_

#include "../utility/ring_buffer.hpp"

/**
 * @class SCIPort
 * @brief Class to emulate SCI(Serial Communication Interface) communication port
 * @details Compatible with anything that performs data communication (UART, I2C, SPI).
 * The distinction of the area should be done where the upper port ID is assigned.
 */
class SCIPort {
 public:
  /**
   * @fn SCIPort
   * @brief Default Constructor. Initialized as default settings.
   */
  SCIPort();
  /**
   * @fn SCIPort
   * @brief Constructor
   * @param [in] rx_buf_size: RX(Component -> OBC) buffer size
   * @param [in] tx_buf_size: TX(OBC -> Component) buffer size
   */
  SCIPort(int rx_buf_size, int tx_buf_size);
  /**
   * @fn ~SCIPort
   * @brief Destructor
   */
  ~SCIPort();

  /**
   * @fn WriteTx
   * @brief Write data to the TX buffer from OBC to Component
   * @param [in] buffer: Data buffer to write
   * @param [in] offset: Start offset of the buffer to write (usually zero)
   * @param [in] count: Length of the data to write
   * @return Number of written byte
   */
  int WriteTx(unsigned char* buffer, int offset, int count);
  /**
   * @fn WriteRx
   * @brief Write data to the RX buffer from Component to OBC
   * @param [in] buffer: Data buffer to write
   * @param [in] offset: Start offset of the buffer to write (usually zero)
   * @param [in] count: Length of the data to write
   * @return Number of written byte
   */
  int WriteRx(unsigned char* buffer, int offset, int count);

  /**
   * @fn ReadTx
   * @brief Read data from the TX buffer by Component
   * @param [out] buffer: Data buffer to stored the read data
   * @param [in] offset: Start offset of the buffer to read (usually zero)
   * @param [in] count: Length of the data to read
   * @return Number of read byte
   */
  int ReadTx(unsigned char* buffer, int offset, int count);
  /**
   * @fn ReadRx
   * @brief Read data from the TX buffer by OBC
   * @param [out] buffer: Data buffer to stored the read data
   * @param [in] offset: Start offset of the buffer to read (usually zero)
   * @param [in] count: Length of the data to read
   * @return Number of read byte
   */
  int ReadRx(unsigned char* buffer, int offset, int count);

 private:
  const static int kDefaultBufferSize = 1024;  //!< Default buffer size

  RingBuffer* rxb_;  //!< Receive buffer (Component -> OBC)
  RingBuffer* txb_;  //!< Transmit buffer (OBC -> Component)
};

#endif  // S2E_INTERFACE_SILS_PORTS_UART_PORT_HPP_
