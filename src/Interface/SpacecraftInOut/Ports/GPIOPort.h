/**
 * @file GPIOPort.h
 * @brief Class to emulate GPIO(General Purpose Input and Output) port
 */

#pragma once
#include <Component/Abstract/IGPIOCompo.h>

#define GPIO_HIGH true
#define GPIO_LOW false

/**
 * @class GPIOPort
 * @brief Class to emulate GPIO(General Purpose Input and Output) port
 */
class GPIOPort {
 public:
  /**
   * @fn GPIOPort
   * @brief Constructor
   * @param [in] port_id_: ID of the GPIO port
   * @param [in] compo: Component which has the GPIO port
   */
  GPIOPort(int port_id_, IGPIOCompo* compo = nullptr);
  /**
   * @fn ~GPIOPort
   * @brief Destructor
   */
  ~GPIOPort();

  /**
   * @fn DigitalWrite
   * @brief Change the GPIO state
   * @param [in] isHigh: Use GPIO_HIGH or GPIO_LOW
   * @return always zero
   */
  int DigitalWrite(bool isHigh);

  /**
   * @fn DigitalRead
   * @brief Read the GPIO state
   * @return GPIO_HIGH or GPIO_LOW
   */
  bool DigitalRead();

 private:
  const int kPortId;       //!< Port ID
  IGPIOCompo* component_;  //!< Component which has the GPIO port
  bool hl_state_;          //!< GPIO High/Low state
};
