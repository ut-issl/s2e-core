/**
 * @file I2cControllerCommunicationBase.h
 * @brief This class simulates the I2C Controller communication with the I2C Target.
 */
#pragma once
#include "../../interface/HilsInOut/HilsPortManager.h"
#include "ObcCommunicationBase.h"

/**
 * @class I2cControllerCommunicationBase
 * @brief This class simulates the I2C Controller communication with the I2C Target.
 * @note Generally, I2C controller side is OBC, and components are target side.
 *       The main purpose is to validate the emulated I2C Target component in the HILS test.
 *       This class works only HILS mode
 */
class I2cControllerCommunicationBase {
 public:
  /**
   * @fn I2cControllerCommunicationBase
   * @brief Constructor
   * @param [in] hils_port_id: ID of HILS communication port
   * @param [in] baud_rate: Baud rate of HILS communication port
   * @param [in] tx_buf_size: TX (Controller to Target) buffer size
   * @param [in] rx_buf_size: RX (Target to Controller) buffer size
   * @param [in] hils_port_manager: HILS port manager
   */
  I2cControllerCommunicationBase(const unsigned int hils_port_id, const unsigned int baud_rate, const unsigned int tx_buf_size,
                                 const unsigned int rx_buf_size, HilsPortManager* hils_port_manager);
  /**
   * @fn ~I2cControllerCommunicationBase
   * @brief Destructor
   */
  ~I2cControllerCommunicationBase();

 protected:
  /**
   * @fn ReceiveTelemetry
   * @brief Receive telemetry
   * @note This function works only HILS mode
   * @param [in] len: Length of data
   */
  int ReceiveTelemetry(const unsigned char len);
  /**
   * @fn SendCommand
   * @brief Send command
   * @note This function works only HILS mode
   * @param [in] len: Length of data
   */
  int SendCommand(const unsigned char len);

  std::vector<unsigned char> tx_buffer_;  //!< TX (Controller to Target) buffer
  std::vector<unsigned char> rx_buffer_;  //!< RX (Target to Controller) buffer

 private:
  unsigned int hils_port_id_;                                   //!< ID of HILS communication port
  int baud_rate_;                                               //!< Baud rate of HILS communication port ex. 9600, 115200
  unsigned int tx_buf_size_;                                    //!< TX (Controller to Target) buffer size
  unsigned int rx_buf_size_;                                    //!< RX (Target to Controller) buffer size
  OBC_COM_UART_MODE sim_mode_ = OBC_COM_UART_MODE::MODE_ERROR;  //!< Simulation mode (SILS or HILS)

  HilsPortManager* hils_port_manager_;  //!< HILS port manager
};
