/**
 * @file IGPIOCompo.h
 * @brief Interface class for components which have GPIO port
 */

#pragma once
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
   * @param[in] isPosedge: Flag to express positive edge or not
   */
  virtual void GPIOStateChanged(int port_id, bool isPosedge) = 0;
};
