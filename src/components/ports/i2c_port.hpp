/**
 * @file i2c_port.hpp
 * @brief Class to emulate I2C(Inter-Integrated Circuit) communication port
 */

#ifndef S2E_COMPONENTS_PORTS_I2C_PORT_HPP_
#define S2E_COMPONENTS_PORTS_I2C_PORT_HPP_

#include <map>

const int kDefaultCmdBufferSize = 0xff;  //!< Default command buffer size

/**
 * @class I2CPort
 * @brief Class to emulate I2C(Inter-Integrated Circuit) communication port
 * @details The class has the register to store the parameters
 */
class I2CPort {
 public:
  /**
   * @fn I2CPort
   * @brief Default Constructor. Nothing happened.
   */
  I2CPort(void);
  /**
   * @fn I2CPort
   * @brief Constructor. Just set the max_register_number.
   * @param [in] max_register_number: Maximum register number
   */
  I2CPort(const unsigned char max_register_number);

  /**
   * @fn RegisterDevice
   * @brief Register the device as an I2C port.
   * @param [in] i2c_address: I2C address
   */
  void RegisterDevice(const unsigned char i2c_addr);

  /**
   * @fn WriteRegister
   * @brief Set the register address to write a value in the next step
   * @param [in] i2c_addr: I2C address of the target device
   * @param [in] reg_addr: Register address to write a value in the next step
   * @return Return zero when an error is happened.
   */
  int WriteRegister(const unsigned char i2c_addr, const unsigned char reg_addr);
  /**
   * @fn WriteRegister
   * @brief Write a value in the target device's register
   * @param [in] i2c_addr: I2C address of the target device
   * @param [in] reg_addr: Register address of the target device
   * @param [in] value: 1 Byte value
   * @return Return zero when an error is happened.
   */
  int WriteRegister(const unsigned char i2c_addr, const unsigned char reg_addr, const unsigned char value);
  /**
   * @fn WriteRegister
   * @brief Write a value in the target device's register
   * @param [in] i2c_addr: I2C address of the target device
   * @param [in] reg_addr: Register address of the target device
   * @param [in] value: float value
   */
  // int WriteRegister(const unsigned char i2c_addr, const unsigned char reg_addr, float value);  // TODO Check this works well

  /**
   * @fn ReadRegister
   * @brief Read the register value of the target device. The register address is used as the previous accessed address
   * @param [in] i2c_addr: I2C address of the target device
   * @return Read data
   */
  unsigned char ReadRegister(const unsigned char i2c_addr);
  /**
   * @fn ReadRegister
   * @brief Read the register value of the target device.
   * @param [in] i2c_addr: I2C address of the target device
   * @param [in] reg_addr: Register address of the target device
   * @return Read data
   */
  unsigned char ReadRegister(const unsigned char i2c_addr, const unsigned char reg_addr);

  // OBC->Component Command emulation
  /**
   * @fn WriteCommand
   * @brief Write command requested from an OBC to the component
   * @param [in] i2c_addr: I2C address of the target device
   * @param [in] tx_data: data from the OBC
   * @param [in] length: length of the tx_data
   * @return Length or zero when an error happened
   */
  unsigned char WriteCommand(const unsigned char i2c_addr, const unsigned char* tx_data, const unsigned char length);
  /**
   * @fn ReadCommand
   * @brief Read command requested from an OBC to the component
   * @param [in] i2c_addr: I2C address of the target device
   * @param [out] rx_data: Data to the OBC
   * @param [in] length: Length of the tx_data
   * @return Length or zero when an error happened
   */
  unsigned char ReadCommand(const unsigned char i2c_addr, unsigned char* rx_data, const unsigned char length);

 private:
  unsigned char max_register_number_ = 0xff;  //!< Maximum register number
  unsigned char saved_reg_addr_ = 0x00;       //!< Saved register address

  /** @brief Device register: <pair(i2c_address, register_address), value>  **/
  std::map<std::pair<unsigned char, unsigned char>, unsigned char> device_registers_;

  /** @brief Buffer for the command from OBC : <pair(i2c_address, cmd_buffer_length), value>  **/
  std::map<std::pair<unsigned char, unsigned char>, unsigned char> cmd_buffer_;
};

#endif  // S2E_COMPONENTS_PORTS_I2C_PORT_HPP_
