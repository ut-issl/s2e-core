/*
 * @file on_board_computer_with_c2a.hpp
 * @brief Class to emulate on board computer with C2A flight software
 */

#ifndef S2E_COMPONENTS_REAL_CDH_OBC_C2A_HPP_
#define S2E_COMPONENTS_REAL_CDH_OBC_C2A_HPP_

#include <components/ports/gpio_port.hpp>

#include "c2a_communication.hpp"
#include "on_board_computer.hpp"

namespace s2e::components {

/*
 * @class ObcWithC2a
 * @brief Class to emulate on board computer with C2A flight software
 */
class ObcWithC2a : public OnBoardComputer {
 public:
  /**
   * @fn ObcWithC2a
   * @brief Constructor
   * @param [in] clock_generator: Clock generator
   */
  ObcWithC2a(ClockGenerator* clock_generator);
  /**
   * @fn ObcWithC2a
   * @brief Constructor
   * @param [in] clock_generator: Clock generator
   * @param [in] timing_regulator: Timing regulator to update flight software faster than the component update
   */
  ObcWithC2a(ClockGenerator* clock_generator, int timing_regulator);
  /**
   * @fn ObcWithC2a
   * @brief Constructor
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_generator: Clock generator
   * @param [in] timing_regulator: Timing regulator to update flight software faster than the component update
   * @param [in] power_port: Power port
   */
  ObcWithC2a(int prescaler, ClockGenerator* clock_generator, int timing_regulator, PowerPort* power_port);
  /**
   * @fn ~ObcWithC2a
   * @brief Destructor
   */
  ~ObcWithC2a();

  // UART Communication port functions. TODO:Rename the following functions to UartHogeHoge
  /**
   * @fn ConnectComPort
   * @brief Connect UART communication port between OnBoardComputer and a component
   * @param [in] port_id: Port ID
   * @param [in] tx_buffer_size: TX (OBC-> Component) buffer size
   * @param [in] rx_buffer_size: RX (Component -> OBC) buffer size
   * @return -1: error, 0: success
   */
  int ConnectComPort(int port_id, int tx_buffer_size, int rx_buffer_size) override;
  /**
   * @fn CloseComPort
   * @brief Close UART communication port between OnBoardComputer and a component
   * @param [in] port_id: Port ID
   * @return -1: error, 0: success
   */
  int CloseComPort(int port_id) override;
  /**
   * @fn SendFromObc
   * @brief Send data from OBC to Components with UART used ny OBC side.
   * @param [in] port_id: Port ID
   * @param [in] buffer: Send data buffer
   * @param [in] offset: Data offset for the buffer
   * @param [in] length: Length of send data
   * @return Number of written byte
   */
  int SendFromObc(int port_id, unsigned char* buffer, int offset, int length) override;
  /**
   * @fn ReceivedByCompo
   * @brief Read data from OBC to Components with UART used by component side.
   * @param [in] port_id: Port ID
   * @param [out] buffer: Read data buffer
   * @param [in] offset: Data offset for the buffer
   * @param [in] length: Length of read data
   * @return Number of read byte
   */
  int ReceivedByCompo(int port_id, unsigned char* buffer, int offset, int length) override;

  /**
   * @fn SendFromComponent
   * @brief Send data from component to OBC with UART used by component side.
   * @param [in] port_id: Port ID
   * @param [in] buffer: Send data buffer
   * @param [in] offset: Data offset for the buffer
   * @param [in] length: Length of send data
   * @return Number of written byte
   */
  int SendFromCompo(int port_id, unsigned char* buffer, int offset, int length) override;
  /**
   * @fn ReceivedByObc
   * @brief Read data from component to OBC with UART used ny OBC side.
   * @param [in] port_id: Port ID
   * @param [out] buffer: Read data buffer
   * @param [in] offset: Data offset for the buffer
   * @param [in] length: Length of read data
   * @return Number of read byte
   */
  int ReceivedByObc(int port_id, unsigned char* buffer, int offset, int length) override;

  // Static function for C2A
  /**
   * @fn SendFromObc_C2A
   * @brief Send data from OBC to Components with UART used by C2A flight software
   * @param [in] port_id: Port ID
   * @param [in] buffer: Send data buffer
   * @param [in] offset: Data offset for the buffer
   * @param [in] length: Length of send data
   * @return Number of written byte
   */
  static int SendFromObc_C2A(int port_id, unsigned char* buffer, int offset, int length);
  /**
   * @fn ReceivedByObc_C2A
   * @brief Read data from component to OBC with UART used by C2A flight software
   * @param [in] port_id: Port ID
   * @param [out] buffer: Read data buffer
   * @param [in] offset: Data offset for the buffer
   * @param [in] length: Length of read data
   * @return Number of read byte
   */
  static int ReceivedByObc_C2A(int port_id, unsigned char* buffer, int offset, int length);

  // I2C
  /**
   * @fn I2cConnectPort
   * @brief Connect I2C communication port between OnBoardComputer (I2C controller) and a component (I2C target)
   * @note Multiple target can be connected to one port ID
   * @param [in] port_id: Port ID
   * @param [in] i2c_address: I2C address of target device
   * @return 0
   */

  int I2cConnectPort(int port_id, const unsigned char i2c_address) override;
  /**
   * @fn I2cCloseComPort
   * @brief Close I2C communication port between OnBoardComputer and a component
   * @param [in] port_id: Port ID
   * @return -1: error, 0: success
   */
  int I2cCloseComPort(int port_id) override;
  /**
   * @fn I2cComponentWriteRegister
   * @brief Write value in the target device's register
   * @param [in] port_id: Port ID
   * @param [in] i2c_address: I2C address of the target device
   * @param [in] register_address: Register address of the target device
   * @param [in] data: Write data buffer
   * @param [in] length: Length of data
   * @return 0
   */
  int I2cComponentWriteRegister(int port_id, const unsigned char i2c_address, const unsigned char register_address, const unsigned char* data,
                                const unsigned char length) override;
  /**
   * @fn I2cComponentReadRegister
   * @brief Read value in the target device's register
   * @param [in] port_id: Port ID
   * @param [in] i2c_address: I2C address of the target device
   * @param [in] register_address: Register address of the target device
   * @param [out] data: Write data buffer
   * @param [in] length: Length of data
   * @return 0
   */
  int I2cComponentReadRegister(int port_id, const unsigned char i2c_address, const unsigned char register_address, unsigned char* data,
                               const unsigned char length) override;
  /**
   * @fn I2cComponentReadCommand
   * @brief Read command from OBC to target device's register
   * @param [in] port_id: Port ID
   * @param [in] i2c_address: I2C address of the target device
   * @param [out] data: Write data buffer
   * @param [in] length: Length of data
   * @return 0
   */
  int I2cComponentReadCommand(int port_id, const unsigned char i2c_address, unsigned char* data, const unsigned char length) override;

  // Static function for C2A
  /**
   * @fn I2cWriteCommand
   * @brief Write command to target device used in C2A flight software
   * @param [in] port_id: Port ID
   * @param [in] i2c_address: I2C address of the target device
   * @param [in] data: Write data buffer
   * @param [in] length: Length of data
   * @return 0
   */
  static int I2cWriteCommand(int port_id, const unsigned char i2c_address, const unsigned char* data, const unsigned char length);
  /**
   * @fn I2cWriteRegister
   * @brief Write value in the target device's register used in C2A flight software
   * @param [in] port_id: Port ID
   * @param [in] i2c_address: I2C address of the target device
   * @param [in] data: Write data buffer
   * @param [in] length: Length of data
   * @return 0
   */
  static int I2cWriteRegister(int port_id, const unsigned char i2c_address, const unsigned char* data, const unsigned char length);
  /**
   * @fn I2cComponentReadRegister
   * @brief Read value in the target device's register used in C2A flight software
   * @param [in] port_id: Port ID
   * @param [in] i2c_address: I2C address of the target device
   * @param [out] data: Write data buffer
   * @param [in] length: Length of data
   * @return 0
   */
  static int I2cReadRegister(int port_id, const unsigned char i2c_address, unsigned char* data, const unsigned char length);

  // GPIO
  /**
   * @fn GpioConnectPort
   * @brief Connect GPIO communication port between OnBoardComputer and a component
   * @param [in] port_id: Port ID
   * @return -1: error, 0: success
   */
  int GpioConnectPort(int port_id) override;
  /**
   * @fn GpioComponentWrite
   * @brief Control GPIO state
   * @param [in] port_id: Port ID
   * @param [in] is_high: GPIO state
   * @return -1: error, 0: success
   */
  int GpioComponentWrite(int port_id, const bool is_high) override;
  /**
   * @fn GpioComponentRead
   * @brief Read GPIO state
   * @param [in] port_id: Port ID
   * @return GPIO state or return false when the port_id is not used
   */
  bool GpioComponentRead(int port_id) override;
  /**
   * @fn GpioWrite_C2A
   * @brief Control GPIO state used in C2A flight software
   * @param [in] port_id: Port ID
   * @param [in] is_high: GPIO state
   * @return -1: error, 0: success
   */
  static int GpioWrite_C2A(int port_id, const bool is_high);
  /**
   * @fn GpioRead_C2A
   * @brief Read GPIO state used in C2A flight software
   * @param [in] port_id: Port ID
   * @return GPIO state or return false when the port_id is not used
   */
  static bool GpioRead_C2A(int port_id);

 private:
  bool is_initialized = false;  //!< Is initialized flag
  const int timing_regulator_;  //!< Timing regulator to update flight software faster than the component update

  // Override functions for Component
  /**
   * @fn MainRoutine
   * @brief Main routine to execute C2A
   */
  void MainRoutine(const int time_count) override;
  /**
   * @fn Initialize
   * @brief Initialize function
   */
  void Initialize() override;

  static std::map<int, UartPort*> com_ports_c2a_;     //!< UART ports
  static std::map<int, I2cPort*> i2c_com_ports_c2a_;  //!< I2C ports
  static std::map<int, GpioPort*> gpio_ports_c2a_;    //!< GPIO ports
};

} // namespace s2e::components

#endif  // S2E_COMPONENTS_REAL_CDH_OBC_C2A_HPP_
