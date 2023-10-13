/**
 * @file example_serial_communication_for_hils.hpp
 * @brief Example of component emulation for UART communication in HILS environment
 */

#ifndef S2E_COMPONENTS_EXAMPLES_EXAMPLE_SERIAL_COMMUNICATION_FOR_HILS_HPP_
#define S2E_COMPONENTS_EXAMPLES_EXAMPLE_SERIAL_COMMUNICATION_FOR_HILS_HPP_

#include <vector>

#include "../base/component.hpp"
#include "../base/uart_communication_with_obc.hpp"

/**
 * @class ExampleSerialCommunicationForHils
 * @brief Example of component emulation for communication in HILS environment
 * @details The sender mode: ExpHils sends out a new massage
 *          - message size is 4 bytes
 *            - the first 3 bytes : ASCII(ABC, BCD, CDE,...)
 *            - the last byte : \0
 *          The responder mode: ExpHils returns the message as received
 */
class ExampleSerialCommunicationForHils : public Component, public UartCommunicationWithObc {
 public:
  /**
   * @fn ExampleSerialCommunicationForHils
   * @brief Constructor
   * @note prescaler is set as 300.
   * @param [in] clock_generator: Clock generator
   * @param [in] sils_port_id: Port ID for communication line b/w OnBoardComputer
   * @param [in] obc: The communication target OBC
   * @param [in] hils_port_id: ID of HILS communication port
   * @param [in] baud_rate: Baud rate of HILS communication port
   * @param [in] hils_port_manager: HILS port manager
   * @param [in] mode_id: Mode ID to select sender(0) or responder(1)
   */
  ExampleSerialCommunicationForHils(ClockGenerator* clock_generator, const int sils_port_id, OnBoardComputer* obc, const unsigned int hils_port_id,
                                    const unsigned int baud_rate, HilsPortManager* hils_port_manager, const int mode_id);
  /**
   * @fn ~ExampleSerialCommunicationForHils
   * @brief Destructor
   */
  ~ExampleSerialCommunicationForHils();

 protected:
  // Override functions for Component
  /**
   * @fn MainRoutine
   * @brief Main routine to receive command and send telemetry
   */
  void MainRoutine(const int time_count) override;

 private:
  const static int kMemorySize = 4;             //!< Memory size
  const int kNumAlphabet = 26;                  //!< Number of Alphabet
  char memory_[kMemorySize] = {0, 0, 0, '\0'};  //!< Memory
  char tx_[kMemorySize] = {0, 0, 0, '\0'};      //!< TX(Telemetry send) buffer
  const int mode_id_;                           //!< Mode ID to select sender(0) or responder(1)
  int counter_ = 0;                             //!< Internal counter

  // Override functions for ObcCommunication
  /**
   * @fn ParseCommand
   * @brief Parse command received from OnBoardComputer
   */
  int ParseCommand(const int command_size) override;
  /**
   * @fn GenerateTelemetry
   * @brief Generate telemetry send to OBC
   */
  int GenerateTelemetry() override;
};

#endif  // S2E_COMPONENTS_EXAMPLES_EXAMPLE_SERIAL_COMMUNICATION_FOR_HILS_HPP_
