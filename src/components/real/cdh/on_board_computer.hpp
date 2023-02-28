/*
 * @file on_board_computer.hpp
 * @brief Class to emulate on board computer
 */

#ifndef S2E_COMPONENTS_REAL_CDH_OBC_HPP_
#define S2E_COMPONENTS_REAL_CDH_OBC_HPP_

#include <components/ports/gpio_port.hpp>
#include <components/ports/i2c_port.hpp>
#include <components/ports/uart_port.hpp>
#include <map>

#include "../../base/component.hpp"

/*
 * @class OBC
 * @brief Class to emulate on board computer
 * @note OBC is connected with other components to communicate, and flight software is executed in OBC.
 */
class OBC : public Component {
 public:
  /**
   * @fn OBC
   * @brief Constructor
   * @param [in] clock_generator: Clock generator
   */
  OBC(ClockGenerator* clock_generator);
  /**
   * @fn OBC
   * @brief Constructor
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_generator: Clock generator
   * @param [in] power_port: Power port
   */
  OBC(int prescaler, ClockGenerator* clock_generator, PowerPort* power_port);
  /**
   * @fn OBC
   * @brief Constructor
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_generator: Clock generator
   * @param [in] power_port: Power port
   * @param [in] minimum_voltage_V: Minimum voltage [V]
   * @param [in] assumed_power_consumption_W: Assumed power consumption [W]
   */
  OBC(int prescaler, ClockGenerator* clock_generator, PowerPort* power_port, const double minimum_voltage_V,
      const double assumed_power_consumption_W);
  /**
   * @fn ~OBC
   * @brief Destructor
   */
  virtual ~OBC();

  // UART Communication port functions. TODO:Rename the following functions to UartHogeHoge
  /**
   * @fn ConnectComPort
   * @brief Connect UART communication port between OBC and a component
   * @param [in] port_id: Port ID
   * @param [in] tx_buffer_size: TX (OBC -> Component) buffer size
   * @param [in] rx_buffer_size: RX (Component -> OBC) buffer size
   * @return -1: error, 0: success
   */
  virtual int ConnectComPort(int port_id, int tx_buffer_size, int rx_buffer_size);
  /**
   * @fn ConnectComPort
   * @brief Close UART communication port between OBC and a component
   * @param [in] port_id: Port ID
   * @return -1: error, 0: success
   */
  virtual int CloseComPort(int port_id);
  /**
   * @fn SendFromObc
   * @brief Send data from OBC to Components with UART used by OBC side.
   * @param [in] port_id: Port ID
   * @param [in] buffer: Send data buffer
   * @param [in] offset: Data offset for the buffer
   * @param [in] length: Length of send data
   * @return Number of written byte
   */
  virtual int SendFromObc(int port_id, unsigned char* buffer, int offset, int length);
  /**
   * @fn ReceivedByCompo
   * @brief Read data from OBC to Components with UART used by component side.
   * @param [in] port_id: Port ID
   * @param [out] buffer: Read data buffer
   * @param [in] offset: Data offset for the buffer
   * @param [in] length: Length of read data
   * @return Number of read byte
   */
  virtual int ReceivedByCompo(int port_id, unsigned char* buffer, int offset, int length);

  /**
   * @fn SendFromComponent
   * @brief Send data from component to OBC with UART used by component side.
   * @param [in] port_id: Port ID
   * @param [in] buffer: Send data buffer
   * @param [in] offset: Data offset for the buffer
   * @param [in] length: Length of send data
   * @return Number of written byte
   */
  virtual int SendFromCompo(int port_id, unsigned char* buffer, int offset, int length);
  /**
   * @fn ReceivedByObc
   * @brief Read data from component to OBC with UART used by OBC side.
   * @param [in] port_id: Port ID
   * @param [out] buffer: Read data buffer
   * @param [in] offset: Data offset for the buffer
   * @param [in] length: Length of read data
   * @return Number of read byte
   */
  virtual int ReceivedByObc(int port_id, unsigned char* buffer, int offset, int length);

  // I2C Communication port functions
  /**
   * @fn I2cConnectPort
   * @brief Connect I2C communication port between OBC (I2C controller) and a component (I2C target)
   * @note Multiple target can be connected to one port ID
   * @param [in] port_id: Port ID
   * @param [in] i2c_address: I2C address of target device
   * @return 0
   */
  virtual int I2cConnectPort(int port_id, const unsigned char i2c_address);
  /**
   * @fn I2cCloseComPort
   * @brief Close I2C communication port between OBC and a component
   * @param [in] port_id: Port ID
   * @return -1: error, 0: success
   */
  virtual int I2cCloseComPort(int port_id);
  /**
   * @fn I2cComponentWriteRegister
   * @brief Write value in the target device's register
   * @param [in] port_id: Port ID
   * @param [in] i2c_address: I2C address of the target device
   * @param [in] reg_address: Register address of the target device
   * @param [in] data: Write data buffer
   * @param [in] length: Length of data
   * @return 0
   */
  virtual int I2cComponentWriteRegister(int port_id, const unsigned char i2c_address, const unsigned char reg_address, const unsigned char* data,
                                        const unsigned char length);
  /**
   * @fn I2cComponentReadRegister
   * @brief Read value in the target device's register
   * @param [in] port_id: Port ID
   * @param [in] i2c_address: I2C address of the target device
   * @param [in] reg_address: Register address of the target device
   * @param [out] data: Write data buffer
   * @param [in] length: Length of data
   * @return 0
   */
  virtual int I2cComponentReadRegister(int port_id, const unsigned char i2c_address, const unsigned char reg_address, unsigned char* data,
                                       const unsigned char length);
  /**
   * @fn I2cComponentReadCommand
   * @brief Read command from OBC to target device's register
   * @param [in] port_id: Port ID
   * @param [in] i2c_address: I2C address of the target device
   * @param [out] data: Write data buffer
   * @param [in] length: Length of data
   * @return 0
   */
  virtual int I2cComponentReadCommand(int port_id, const unsigned char i2c_address, unsigned char* data, const unsigned char length);

  // GPIO port functions
  /**
   * @fn GpioConnectPort
   * @brief Connect GPIO communication port between OBC and a component
   * @param [in] port_id: Port ID
   * @return -1: error, 0: success
   */
  virtual int GpioConnectPort(int port_id);
  /**
   * @fn GpioComponentWrite
   * @brief Control GPIO state
   * @param [in] port_id: Port ID
   * @param [in] is_high: GPIO state
   * @return -1: error, 0: success
   */
  virtual int GpioComponentWrite(int port_id, const bool is_high);
  /**
   * @fn GpioComponentRead
   * @brief Read GPIO state
   * @param [in] port_id: Port ID
   * @return GPIO state or return false when the port_id is not used
   */
  virtual bool GpioComponentRead(int port_id);

 protected:
  /**
   * @fn Initialize
   * @brief Initialize function
   */
  virtual void Initialize();

  // Override functions for Component
  /**
   * @fn MainRoutine
   * @brief Main routine to execute flight software
   * @note Users need to write flight software
   */
  virtual void MainRoutine(int time_count);

 private:
  std::map<int, UartPort*> uart_ports_;  //!< UART ports
  std::map<int, I2cPort*> i2c_ports_;    //!< I2C ports
  std::map<int, GpioPort*> gpio_ports_;  //!< GPIO ports
};

#endif  // S2E_COMPONENTS_REAL_CDH_OBC_HPP_
