/**
 * @file uart_port.hpp
 * @brief Class to emulate UART communication port
 */

#ifndef S2E_COMPONENTS_PORTS_UART_PORT_HPP_
#define S2E_COMPONENTS_PORTS_UART_PORT_HPP_

#include <library/utilities/ring_buffer.hpp>

/**
 * @class UartPort
 * @brief Class to emulate SCI(Serial Communication Interface) communication port
 * @details Compatible with anything that performs data communication (UART, I2C, SPI).
 * The distinction of the area should be done where the upper port ID is assigned.
 */
class UartPort {
 public:
  /**
   * @fn UartPort
   * @brief Default Constructor. Initialized as default settings.
   */
  UartPort();
  /**
   * @fn UartPort
   * @brief Constructor
   * @param [in] rx_buffer_size: RX(Component -> OBC) buffer size
   * @param [in] tx_buffer_size: TX(OBC -> Component) buffer size
   */
  UartPort(const unsigned int rx_buffer_size, const unsigned int tx_buffer_size);
  /**
   * @fn ~UartPort
   * @brief Destructor
   */
  ~UartPort();

  /**
   * @fn WriteTx
   * @brief Write data to the TX buffer from OBC to Component
   * @param [in] buffer: Data buffer to write
   * @param [in] offset: Start offset of the buffer to write (usually zero)
   * @param [in] data_length: Length of the data to write
   * @return Number of written byte
   */
  int WriteTx(const unsigned char* buffer, const unsigned int offset, const unsigned int data_length);
  /**
   * @fn WriteRx
   * @brief Write data to the RX buffer from Component to OBC
   * @param [in] buffer: Data buffer to write
   * @param [in] offset: Start offset of the buffer to write (usually zero)
   * @param [in] data_length: Length of the data to write
   * @return Number of written byte
   */
  int WriteRx(const unsigned char* buffer, const unsigned int offset, const unsigned int data_length);

  /**
   * @fn ReadTx
   * @brief Read data from the TX buffer by Component
   * @param [out] buffer: Data buffer to stored the read data
   * @param [in] offset: Start offset of the buffer to read (usually zero)
   * @param [in] data_length: Length of the data to read
   * @return Number of read byte
   */
  int ReadTx(unsigned char* buffer, const unsigned int offset, const unsigned int data_length);
  /**
   * @fn ReadRx
   * @brief Read data from the TX buffer by OBC
   * @param [out] buffer: Data buffer to stored the read data
   * @param [in] offset: Start offset of the buffer to read (usually zero)
   * @param [in] data_length: Length of the data to read
   * @return Number of read byte
   */
  int ReadRx(unsigned char* buffer, const unsigned int offset, const unsigned int data_length);

 private:
  const static unsigned int kDefaultBufferSize = 1024;  //!< Default buffer size

  RingBuffer* rx_buffer_;  //!< Receive buffer (Component -> OBC)
  RingBuffer* tx_buffer_;  //!< Transmit buffer (OBC -> Component)
};

#endif  // S2E_COMPONENTS_PORTS_UART_PORT_HPP_
