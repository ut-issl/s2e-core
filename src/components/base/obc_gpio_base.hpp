/**
 * @file obc_gpio_base.hpp
 * @brief Base class for GPIO communication with OBC flight software
 *        TODO: consider relation with IGPIOCompo
 */

#ifndef S2E_COMPONENTS_BASE_CLASSES_OBC_GPIO_BASE_HPP_
#define S2E_COMPONENTS_BASE_CLASSES_OBC_GPIO_BASE_HPP_

#include "../cdh/obc.hpp"

/**
 * @class ObcGpioBase
 * @brief Base class for GPIO communication with OBC flight software
 * @note Components which want to communicate with OBC using GPIO have to inherit this.
 */
class ObcGpioBase {
 public:
  /**
   * @fn ObcGpioBase
   * @brief Constructor for SILS mode
   * @param [in] port_id: Port ID GPIO line
   * @param [in] obc: The communication target OBC
   */
  ObcGpioBase(const std::vector<int> port_id, OBC* obc);
  /**
   * @fn ~ObcGpioBase
   * @brief Destructor
   */
  ~ObcGpioBase();

 protected:
  /**
   * @fn Read
   * @brief Read the GPIO state
   * @param [in] idx: element index for port_id_ vector, not the GPIO port ID for OBC.
   * @return High(True) or Low(False) of GPIO state
   */
  bool Read(const int idx);
  /**
   * @fn Write
   * @brief Write the GPIO state
   * @param [in] idx: element index for port_id_ vector, not the GPIO port ID for OBC.
   * @param [in] is_high: High(True) or Low(False) of GPIO state
   */
  void Write(const int idx, const bool is_high);

 private:
  std::vector<int> port_id_;  //!< Port ID GPIO line
  OBC* obc_;                  //!< The communication target OBC
};

#endif  // S2E_COMPONENTS_BASE_CLASSES_OBC_GPIO_BASE_HPP_
