/**
 * @file example_serial_communication_with_obc.hpp
 * @brief Example of component emulation with communication between OnBoardComputer Flight software
 */

#ifndef S2E_COMPONENTS_EXAMPLES_EXAMPLE_SERIAL_COMMUNICATION_WITH_OBC_HPP_P_
#define S2E_COMPONENTS_EXAMPLES_EXAMPLE_SERIAL_COMMUNICATION_WITH_OBC_HPP_P_

#include <vector>

#include "../base/component.hpp"
#include "../base/interface_gpio_component.hpp"
#include "../base/uart_communication_with_obc.hpp"

/**
 * @class ExampleSerialCommunicationWithObc
 * @brief Example of component emulation with communication between OnBoardComputer Flight software
 * @details Command to EXP is 5 bytes.
 *          - The first 3 bytes: "SET"
 *          - The Fourth byte: Set data (ASCII 0x21~0x7e)
 *          - The last byte: "\n"
 *          Telemetry from EXP is 100 bytes.
 *          - Send the accumulated data set by the SET command.
 *          - Unset data is filled with '\0'
 *          - The last byte: "\n"
 *          - The telemetry is automatically generated
 */
class ExampleSerialCommunicationWithObc : public Component, public UartCommunicationWithObc, public IGPIOCompo {
 public:
  /**
   * @fn ExampleSerialCommunicationWithObc
   * @brief Constructor without prescaler
   * @note The prescaler is set as 1000
   * @param [in] clock_generator: Clock generator
   * @param [in] port_id: Port ID for communication line b/w OnBoardComputer
   * @param [in] obc: The communication target OBC
   */
  ExampleSerialCommunicationWithObc(ClockGenerator* clock_generator, int port_id, OnBoardComputer* obc);
  /**
   * @fn ExampleSerialCommunicationWithObc
   * @brief Constructor
   * @param [in] clock_generator: Clock generator
   * @param [in] port_id: Port ID for communication line b/w OnBoardComputer
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] obc: The communication target OBC
   */
  ExampleSerialCommunicationWithObc(ClockGenerator* clock_generator, int port_id, int prescaler, OnBoardComputer* obc);
  /**
   * @fn ~SerialCommunicationWithObc
   * @brief Destructor
   */
  ~ExampleSerialCommunicationWithObc();

 protected:
  // Override functions for Component
  /**
   * @fn MainRoutine
   * @brief Main routine to receive command and send telemetry
   */
  void MainRoutine(const int time_count);

  // Override functions for IGPIOCompo
  /**
   * @fn GpioStateChanged
   * @brief Interrupt function for GPIO
   */
  void GpioStateChanged(int port_id, bool isPosedge);

 private:
  const static int kMaxMemoryLength = 100;  //!< Maximum memory length
  std::vector<char> memory_;                //!< Memory for telemetry generation

  // Override functions for ObcCommunication
  /**
   * @fn ParseCommand
   * @brief Parse command received from OnBoardComputer
   */
  int ParseCommand(const int command_size) override;
  /**
   * @fn GenerateTelemetry
   * @brief Generate telemetry send to OnBoardComputer
   */
  int GenerateTelemetry() override;

  /**
   * @fn Initialize
   * @brief Initialize function
   */
  int Initialize();
};

#endif  // S2E_COMPONENTS_EXAMPLES_EXAMPLE_SERIAL_COMMUNICATION_WITH_OBC_HPP_P_
