/**
 * @file interface_gpio_component.hpp
 * @brief Interface class for components which have GPIO port
 */

#ifndef S2E_COMPONENTS_BASE_CLASSES_INTERFACE_GPIO_COMPONENT_HPP_
#define S2E_COMPONENTS_BASE_CLASSES_INTERFACE_GPIO_COMPONENT_HPP_

/**
 * @class IGPIOCompo
 * @brief Interface class for components which have GPIO port
 */
class IGPIOCompo {
 public:
  /**
   * @fn ~IGPIOCompo
   * @brief Destructor
   */
  virtual ~IGPIOCompo(){};

  /**
   * @fn GPIOStateChanged
   * @brief Pure virtual function called at the GPIO state is changed like interrupt function.
   * @param[in] port_id: GPIO port ID
   * @param[in] is_positive_edge: Flag to express positive edge or not
   */
  virtual void GPIOStateChanged(const int port_id, const bool is_positive_edge) = 0;
};

#endif  // S2E_COMPONENTS_BASE_CLASSES_INTERFACE_GPIO_COMPONENT_HPP_
