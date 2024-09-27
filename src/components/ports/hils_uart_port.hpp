/**
 * @file hils_uart_port.hpp
 * @brief Class to manage PC's COM port
 * @details Currently, this feature supports Windows Visual Studio only.(FIXME)
 * Reference: https://docs.microsoft.com/en-us/dotnet/api/system.io.ports.serialport?view=netframework-4.7.2
 * @note TODO :We need to clarify the difference with ComPortInterface
 */

#ifndef S2E_COMPONENTS_PORTS_HILS_UART_PORT_HPP_
#define S2E_COMPONENTS_PORTS_HILS_UART_PORT_HPP_

#include <msclr/gcroot.h>
#include <msclr/marshal_cppstd.h>

#include <string>

namespace s2e::components {

typedef cli::array<System::Byte> bytearray;  //!< System::Byte: an 8-bit unsigned integer

/**
 * @class HilsUartPort
 * @brief Class to manage PC's COM port
 * @note TX means S2E -> COM port -> External device
 * RX means External device -> COM Port -> S2E
 */
class HilsUartPort {
 public:
  /**
   * @fn HilsUartPort
   * @brief Constructor.
   * @param [in] port_id: COM port ID
   * @param [in] baud_rate: Baudrate of the COM port
   * @param [in] tx_buffer_size: TX buffer size
   * @param [in] rx_buffer_size: RX buffer size
   */
  HilsUartPort(const unsigned int port_id, const unsigned int baud_rate, const unsigned int tx_buffer_size, const unsigned int rx_buffer_size);
  /**
   * @fn ~HilsUartPort
   * @brief Destructor.
   */
  ~HilsUartPort();

  /**
   * @fn OpenPort
   * @brief Open the COM port
   */
  int OpenPort();
  /**
   * @fn ClosePort
   * @brief Close the COM port
   */
  int ClosePort();

  /**
   * @fn WriteTx
   * @brief Send data to COM port
   * @param [in] buffer: Data buffer to send
   * @param [in] offset: Start offset for the data buffer to send
   * @param [in] data_length: Length of data to send
   * @return 0: success, -1: error
   */
  int WriteTx(const unsigned char* buffer, const unsigned int offset, const unsigned int data_length);
  /**
   * @fn ReadRx
   * @brief Read data from COM port
   * @param [out] buffer: Data buffer to store read data
   * @param [in] offset: Start offset for the data buffer to read
   * @param [in] data_length: Length of data to read
   * @return received data length: success, negative value: error
   */
  int ReadRx(unsigned char* buffer, const unsigned int offset, const unsigned int data_length);
  /**
   * @fn GetBytesToRead
   * @brief Get length of byte to read
   * @return Length of byte to read or -1 when error happened
   */
  int GetBytesToRead();

 private:
  const unsigned int kTxBufferSize;  //!< TX Buffer size
  const unsigned int kRxBufferSize;  //!< RX Buffer size
  const std::string kPortName;       //!< Port name like "COM4"
  unsigned int baud_rate_;           //!< Baud rate ex. 9600, 115200

  // gcroot is the type-safe wrapper template to refer to a CLR object from the c++ heap reference:
  // https://docs.microsoft.com/en-us/cpp/dotnet/how-to-declare-handles-in-native-types?view=msvc-160
  msclr::gcroot<System::IO::Ports::SerialPort ^> port_;  //!< Port
  msclr::gcroot<bytearray ^> tx_buffer_;                 //!< TX Buffer
  msclr::gcroot<bytearray ^> rx_buffer_;                 //!< RX Buffer

  /**
   * @fn GetPortName
   * @brief Convert port id to port name
   * @param [in] port_id: Port ID like 4
   * @return Port name like "COM4"
   */
  static std::string GetPortName(const unsigned int port_id);
  /**
   * @fn Initialize
   * @brief Open and initialize the COM port
   */
  int Initialize();
  /**
   * @fn DiscardInBuffer
   * @brief Discard in buffer
   */
  int DiscardInBuffer();
  /**
   * @fn DiscardOutBuffer
   * @brief Discard out buffer
   */
  int DiscardOutBuffer();
};

}  // namespace s2e::components

#endif  // S2E_COMPONENTS_PORTS_HILS_UART_PORT_HPP_
