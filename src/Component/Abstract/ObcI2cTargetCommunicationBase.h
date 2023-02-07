/**
 * @file ObcI2cTargetCommunicationBase.h
 * @brief Base class for I2C communication as target side with OBC flight software
 */
#pragma once
#include "../../interface/HilsInOut/HilsPortManager.h"
#include "../CDH/OBC.h"
#include "ObcCommunicationBase.h"

/**
 * @class ObcI2cTargetCommunicationBase
 * @brief Base class for I2C communication as target side with OBC flight software
 * @note Generally, components are the target side of I2C (OBC is the controller side).
 */
class ObcI2cTargetCommunicationBase {
 public:
  /**
   * @fn ObcI2cTargetCommunicationBase
   * @brief Constructor for SILS mode
   * @param [in] sils_port_id: Port ID for communication line b/w OBC in the SILS mode
   * @param [in] i2c_address: I2C address for the target
   * @param [in] obc: The communication target OBC
   */
  ObcI2cTargetCommunicationBase(const unsigned int sils_port_id, const unsigned char i2c_address, OBC* obc);
  /**
   * @fn ObcI2cTargetCommunicationBase
   * @brief Constructor for HILS mode
   * @param [in] hils_port_id: ID of HILS communication port
   * @param [in] i2c_address: I2C address for the target
   * @param [in] hils_port_manager: HILS port manager
   */
  ObcI2cTargetCommunicationBase(const unsigned int hils_port_id, const unsigned char i2c_address, HilsPortManager* hils_port_manager);
  /**
   * @fn ObcI2cTargetCommunicationBase
   * @brief Constructor for both SILS and HILS mode
   * @param [in] sils_port_id: Port ID for communication line b/w OBC in the SILS mode
   * @param [in] hils_port_id: ID of HILS communication port
   * @param [in] i2c_address: I2C address for the target
   * @param [in] obc: The communication target OBC
   * @param [in] hils_port_manager: HILS port manager
   */
  ObcI2cTargetCommunicationBase(const unsigned int sils_port_id, const unsigned int hils_port_id, const unsigned char i2c_address, OBC* obc,
                                HilsPortManager* hils_port_manager);
  /**
   * @fn ObcI2cTargetCommunicationBase
   * @brief Prevent double freeing of memory when this class is copied
   */
  ObcI2cTargetCommunicationBase(ObcI2cTargetCommunicationBase&& obj) noexcept;
  /**
   * @fn ~ObcI2cTargetCommunicationBase
   * @brief Destructor
   */
  ~ObcI2cTargetCommunicationBase();

 protected:
  /**
   * @fn ReadRegister
   * @brief Read register of I2C port
   * @param [in] reg_addr: Address of the target register
   * @param [out] data: Buffer to store the read data
   * @param [in] len: Length of the data
   */
  void ReadRegister(const unsigned char reg_addr, unsigned char* data, const unsigned char len);
  /**
   * @fn WriteRegister
   * @brief Read register of I2C port
   * @param [in] reg_addr: Address of the target register
   * @param [in] data: Write data
   * @param [in] len: Length of the data
   */
  void WriteRegister(const unsigned char reg_addr, const unsigned char* data, const unsigned char len);
  /**
   * @fn ReadCommand
   * @brief Read command from I2C controller
   * @param [out] data: Buffer to store the read command data
   * @param [in] len: Length of the data
   */
  void ReadCommand(unsigned char* data, const unsigned char len);
  /**
   * @fn ReceiveCommand
   * @brief Receive command (HILS only)
   * @note This wraps I2cTargetReceive in HilsPortManager
   */
  int ReceiveCommand();
  /**
   * @fn SendTelemetry
   * @brief Send Telemetry (HILS only)
   * @param [in] len: Data length to write
   * @note This wraps I2cTargetSend in HilsPortManager
   */
  int SendTelemetry(const unsigned char len);
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

  OBC_COM_UART_MODE sim_mode_ = OBC_COM_UART_MODE::MODE_ERROR;  //!< Simulation mode

  OBC* obc_;                            //!< Communication target OBC
  HilsPortManager* hils_port_manager_;  //!< HILS port manager
};
