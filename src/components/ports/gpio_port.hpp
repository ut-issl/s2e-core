/**
 * @file gpio_port.hpp
 * @brief Class to emulate GPIO(General Purpose Input and Output) port
 */

#ifndef S2E_COMPONENTS_PORTS_GPIO_PORT_HPP_
#define S2E_COMPONENTS_PORTS_GPIO_PORT_HPP_

#include <components/base/interface_gpio_component.hpp>

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
  GPIOPort(const unsigned int port_id_, IGPIOCompo* compo = nullptr);
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
  int DigitalWrite(const bool isHigh);

  /**
   * @fn DigitalRead
   * @brief Read the GPIO state
   * @return GPIO_HIGH or GPIO_LOW
   */
  bool DigitalRead();

 private:
  const unsigned int kPortId;  //!< Port ID
  IGPIOCompo* component_;      //!< Component which has the GPIO port
  bool high_low_state_;        //!< GPIO High/Low state
};

#endif  // S2E_COMPONENTS_PORTS_GPIO_PORT_HPP_
