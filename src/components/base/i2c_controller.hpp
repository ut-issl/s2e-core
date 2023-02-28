/**
 * @file i2c_controller.hpp
 * @brief This class simulates the I2C Controller communication with the I2C Target.
 */

#ifndef S2E_COMPONENTS_BASE_I2C_CONTROLLER_HPP_
#define S2E_COMPONENTS_BASE_I2C_CONTROLLER_HPP_

#include "../../simulation/hils/hils_port_manager.hpp"
#include "uart_communication_with_obc.hpp"

/**
 * @class I2cController
 * @brief This class simulates the I2C Controller communication with the I2C Target.
 * @note Generally, I2C controller side is OBC, and components are target side.
 *       The main purpose is to validate the emulated I2C Target component in the HILS test.
 *       This class works only HILS mode
 */
class I2cController {
 public:
  /**
   * @fn I2cController
   * @brief Constructor
   * @param [in] hils_port_id: ID of HILS communication port
   * @param [in] baud_rate: Baud rate of HILS communication port
   * @param [in] tx_buffer_size: TX (Controller to Target) buffer size
   * @param [in] rx_buffer_size: RX (Target to Controller) buffer size
   * @param [in] hils_port_manager: HILS port manager
   */
  I2cController(const unsigned int hils_port_id, const unsigned int baud_rate, const unsigned int tx_buffer_size, const unsigned int rx_buffer_size,
                HilsPortManager* hils_port_manager);
  /**
   * @fn ~I2cController
   * @brief Destructor
   */
  ~I2cController();

 protected:
  /**
   * @fn ReceiveTelemetry
   * @brief Receive telemetry
   * @note This function works only HILS mode
   * @param [in] length: Length of data
   */
  int ReceiveTelemetry(const unsigned char length);
  /**
   * @fn SendCommand
   * @brief Send command
   * @note This function works only HILS mode
   * @param [in] length: Length of data
   */
  int SendCommand(const unsigned char length);

  std::vector<unsigned char> tx_buffer_;  //!< TX (Controller to Target) buffer
  std::vector<unsigned char> rx_buffer_;  //!< RX (Target to Controller) buffer

 private:
  unsigned int hils_port_id_;                                          //!< ID of HILS communication port
  unsigned int baud_rate_;                                             //!< Baud rate of HILS communication port ex. 9600, 115200
  unsigned int tx_buffer_size_;                                        //!< TX (Controller to Target) buffer size
  unsigned int rx_buffer_size_;                                        //!< RX (Target to Controller) buffer size
  SimulationMode simulation_mode_ = SimulationMode::kError;  //!< Simulation mode (SILS or HILS)

  HilsPortManager* hils_port_manager_;  //!< HILS port manager
};

#endif  // S2E_COMPONENTS_BASE_I2C_CONTROLLER_HPP_
