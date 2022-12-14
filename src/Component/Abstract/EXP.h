/**
 * @file EXP.h
 * @brief Example of component emulation with communication between OBC Flight software
 */

#pragma once
#include <vector>

#include "ComponentBase.h"
#include "IGPIOCompo.h"
#include "ObcCommunicationBase.h"

/**
 * @class EXP
 * @brief Example of component emulation with communication between OBC Flight software
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
class EXP : public ComponentBase, public ObcCommunicationBase, public IGPIOCompo {
 public:
  /**
   * @fn EXP
   * @brief Constructor without prescaler
   * @note The prescaler is set as 1000
   * @param [in] clock_gen: Clock generator
   * @param [in] port_id: Port ID for communication line b/w OBC
   * @param [in] obc:The communication target OBC
   */
  EXP(ClockGenerator* clock_gen, int port_id, OBC* obc);
  /**
   * @fn EXP
   * @brief Constructor
   * @param [in] clock_gen: Clock generator
   * @param [in] port_id: Port ID for communication line b/w OBC
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] obc:The communication target OBC
   */
  EXP(ClockGenerator* clock_gen, int port_id, int prescaler, OBC* obc);
  /**
   * @fn ~EXP
   * @brief Destructor
   */
  ~EXP();

 protected:
  // Override functions for ComponentBase
  /**
   * @fn MainRoutine
   * @brief Main routine to receive command and send telemetry
   */
  void MainRoutine(int count);

  // Override functions for IGPIOCompo
  /**
   * @fn GPIOStateChanged
   * @brief Interrupt function for GPIO
   */
  void GPIOStateChanged(int port_id, bool isPosedge);

 private:
  const static int MAX_MEMORY_LEN = 100;  //!< Maximum memory length
  std::vector<char> memory;               //!< Memory for telemetry generation
  char memoryc[100];                      //!< Memory in char array
  unsigned char tx_buff[MAX_MEMORY_LEN];  //!< TX (Telemetry send) buffer
  unsigned char rx_buff[MAX_MEMORY_LEN];  //!< RX (Command receive) buffer

  // Override functions for ObcComunication
  /**
   * @fn ParseCommand
   * @brief Parse command received from OBC
   */
  int ParseCommand(const int cmd_size) override;
  /**
   * @fn GenerateTelemetry
   * @brief Generate telemetry send to OBC
   */
  int GenerateTelemetry() override;

  /**
   * @fn Initialize
   * @brief Initialize function
   */
  int Initialize();
};