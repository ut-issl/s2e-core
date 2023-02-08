/**
 * @file com_port_interface.hpp
 * @brief Class to manage PC's COM port
 * @details Currently, this feature supports Windows Visual Studio only.(FIXME)
 * Reference: https://docs.microsoft.com/ja-jp/dotnet/api/system.io.ports.serialport?view=netframework-4.7.2
 */

#ifndef S2E_INTERFACE_HILS_COM_PORT_INTERFACE_H_
#define S2E_INTERFACE_HILS_COM_PORT_INTERFACE_H_

#include <components/base_classes/interface_tickable.hpp>
#include <msclr/gcroot.h>
#include <msclr/marshal_cppstd.h>

#include <string>

using namespace System;
using namespace System::Runtime::InteropServices;
typedef cli::array<Byte> bytearray;

/**
 * @class ComPortInterface
 * @brief Class to manage PC's COM port
 */
class ComPortInterface {
 public:
  /**
   * @fn ComPortInterface
   * @brief Constructor. This function doesn't open the COM port. Users need to call Initialize function after.
   * @param [in] port_id: COM port ID
   * @param [in] baudrate: Baudrate of the COM port
   * @param [in] tx_buffer_size: TX buffer size
   * @param [in] rx_buffer_size: RX buffer size
   */
  ComPortInterface(int port_id, int baudrate, unsigned int tx_buffer_size, unsigned int rx_buffer_size);
  /**
   * @fn ~ComPortInterface
   * @brief Destructor.
   */
  ~ComPortInterface();

  /**
   * @fn PortName
   * @brief Convert port id to port name
   * @param [in] port_id: Port ID like 4
   * @return Port name like "COM4"
   */
  static std::string PortName(int port_id);

  /**
   * @fn Initialize
   * @brief Open and initialize the COM port
   */
  int Initialize();
  /**
   * @fn Finalize
   * @brief Close and finalize the COM port
   */
  int Finalize();

  /**
   * @fn Send
   * @brief Send data to COM port
   * @param [in] buffer: Data buffer to send
   * @param [in] offset: Start offset for the data buffer to send
   * @param [in] count: Length of data to send
   * @return 0: success, -1: error
   */
  int Send(unsigned char *buffer, size_t offset, size_t count);
  /**
   * @fn Receive
   * @brief Receive data from COM port
   * @param [out] buffer: Data buffer to receive
   * @param [in] offset: Start offset for the data buffer to receive
   * @param [in] count: Length of data to receive
   * @return received data length: success, 0: error
   */
  int Receive(unsigned char *buffer, size_t offset, size_t count);

  /**
   * @fn DiscardInBuffer
   * @brief Discard buffer
   */
  int DiscardInBuffer();
  /**
   * @fn BytesToRead
   * @brief Get length of byte to read
   * @return Length of byte to read or -1 when error happened
   */
  int BytesToRead();

  const size_t kTxBufferSize;   //!< TX buffer size
  const size_t kRxBufferSize;   //!< RX buffer size
  const std::string kPortName;  //!< Port name like "COM4"
  const int kPortId;            //!< Port ID like 4 for "COM4"
  const int kBaudRate;          //<! Baudrate

 private:
  int OpenPort();
  msclr::gcroot<IO::Ports::SerialPort ^> port_;  //!< Port
  // Byte tx_buf_[kTxBufferSize];
  // Byte rx_buf_[kRxBufferSize];
  msclr::gcroot<bytearray ^> tx_buf_;  //!< TX Buffer
  msclr::gcroot<bytearray ^> rx_buf_;  //!< RX Buffer
};

#endif  // S2E_INTERFACE_HILS_COM_PORT_INTERFACE_H_
