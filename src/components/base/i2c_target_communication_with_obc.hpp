/**
 * @file i2c_target_communication_with_obc.hpp
 * @brief Base class for I2C communication as target side with OBC flight software
 */

#ifndef S2E_COMPONENTS_BASE_I2C_TARGET_COMMUNICATION_WITH_OBC_HPP_
#define S2E_COMPONENTS_BASE_I2C_TARGET_COMMUNICATION_WITH_OBC_HPP_

#include "../../simulation/hils/hils_port_manager.hpp"
#include "../real/cdh/on_board_computer.hpp"
#include "uart_communication_with_obc.hpp"

/**
 * @class I2cTargetCommunicationWithObc
 * @brief Base class for I2C communication as target side with OBC flight software
 * @note Generally, components are the target side of I2C (OnBoardComputer is the controller side).
 */
class I2cTargetCommunicationWithObc {
 public:
  /**
   * @fn I2cTargetCommunicationWithObc
   * @brief Constructor for SILS mode
   * @param [in] sils_port_id: Port ID for communication line b/w OnBoardComputer in the SILS mode
   * @param [in] i2c_address: I2C address for the target
   * @param [in] obc: The communication target OnBoardComputer
   */
  I2cTargetCommunicationWithObc(const unsigned int sils_port_id, const unsigned char i2c_address, OnBoardComputer* obc);
  /**
   * @fn I2cTargetCommunicationWithObc
   * @brief Constructor for HILS mode
   * @param [in] hils_port_id: ID of HILS communication port
   * @param [in] i2c_address: I2C address for the target
   * @param [in] hils_port_manager: HILS port manager
   */
  I2cTargetCommunicationWithObc(const unsigned int hils_port_id, const unsigned char i2c_address, HilsPortManager* hils_port_manager);
  /**
   * @fn I2cTargetCommunicationWithObc
   * @brief Constructor for both SILS and HILS mode
   * @param [in] sils_port_id: Port ID for communication line b/w OnBoardComputer in the SILS mode
   * @param [in] hils_port_id: ID of HILS communication port
   * @param [in] i2c_address: I2C address for the target
   * @param [in] obc: The communication target OnBoardComputer
   * @param [in] hils_port_manager: HILS port manager
   */
  I2cTargetCommunicationWithObc(const unsigned int sils_port_id, const unsigned int hils_port_id, const unsigned char i2c_address,
                                OnBoardComputer* obc, HilsPortManager* hils_port_manager);
  /**
   * @fn I2cTargetCommunicationWithObc
   * @brief Prevent double freeing of memory when this class is copied
   */
  I2cTargetCommunicationWithObc(I2cTargetCommunicationWithObc&& object) noexcept;
  /**
   * @fn ~I2cTargetCommunicationWithObc
   * @brief Destructor
   */
  ~I2cTargetCommunicationWithObc();

 protected:
  /**
   * @fn ReadRegister
   * @brief Read register of I2C port
   * @param [in] register_address: Address of the target register
   * @param [out] data: Buffer to store the read data
   * @param [in] length: Length of the data
   */
  void ReadRegister(const unsigned char register_address, unsigned char* data, const unsigned char length);
  /**
   * @fn WriteRegister
   * @brief Read register of I2C port
   * @param [in] register_address: Address of the target register
   * @param [in] data: Write data
   * @param [in] length: Length of the data
   */
  void WriteRegister(const unsigned char register_address, const unsigned char* data, const unsigned char length);
  /**
   * @fn ReadCommand
   * @brief Read command from I2C controller
   * @param [out] data: Buffer to store the read command data
   * @param [in] length: Length of the data
   */
  void ReadCommand(unsigned char* data, const unsigned char length);
  /**
   * @fn ReceiveCommand
   * @brief Receive command (HILS only)
   * @note This wraps I2cTargetReceive in HilsPortManager
   */
  int ReceiveCommand();
  /**
   * @fn SendTelemetry
   * @brief Send Telemetry (HILS only)
   * @param [in] length: Data length to write
   * @note This wraps I2cTargetSend in HilsPortManager
   */
  int SendTelemetry(const unsigned char length);
  /**
   * @fn GetStoredFrameCounter
   * @brief Return stored frame count (HILS only)
   * @note This wraps I2cTargetGetStoredFrameCounter in HilsPortManager
   */
  int GetStoredFrameCounter();
  /**
   * @fn StoreTelemetry
   * @brief Store telemetry in converter up to stored_frame_num (HILS only)
   */
  int StoreTelemetry(const unsigned int stored_frame_num, const unsigned char tlm_size);
  /**
   * @fn GetI2cAddress
   * @brief Return I2C address
   */
  unsigned char GetI2cAddress() const { return i2c_address_; }

 private:
  unsigned int sils_port_id_;  //!< Port ID for SILS
  unsigned int hils_port_id_;  //!< Port ID for HILS
  unsigned char i2c_address_;  //!< I2C address for the target
  bool is_moved_ = false;      //!< Flag to show the object is copied or not

  OBC_COM_UART_MODE simulation_mode_ = OBC_COM_UART_MODE::MODE_ERROR;  //!< Simulation mode

  OnBoardComputer* obc_;                //!< Communication target OnBoardComputer
  HilsPortManager* hils_port_manager_;  //!< HILS port manager
};

#endif  // S2E_COMPONENTS_BASE_I2C_TARGET_COMMUNICATION_WITH_OBC_HPP_
