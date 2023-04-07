/**
 * @file gpio_connection_with_obc.hpp
 * @brief Base class for GPIO communication with OBC flight software
 *        TODO: consider relation with IGPIOCompo
 */

#ifndef S2E_COMPONENTS_BASE_GPIO_CONNECTION_WITH_OBC_HPP_
#define S2E_COMPONENTS_BASE_GPIO_CONNECTION_WITH_OBC_HPP_

#include "../real/cdh/on_board_computer.hpp"

/**
 * @class GpioConnectionWithObc
 * @brief Base class for GPIO communication with OBC flight software
 * @note Components which want to communicate with OBC using GPIO have to inherit this.
 */
class GpioConnectionWithObc {
 public:
  /**
   * @fn GpioConnectionWithObc
   * @brief Constructor for SILS mode
   * @param [in] port_id: Port ID GPIO line
   * @param [in] obc: The communication target OBC
   */
  GpioConnectionWithObc(const std::vector<int> port_id, OnBoardComputer* obc);
  /**
   * @fn ~GpioConnectionWithObc
   * @brief Destructor
   */
  ~GpioConnectionWithObc();

 protected:
  /**
   * @fn Read
   * @brief Read the GPIO state
   * @param [in] index: element index for port_id_ vector, not the GPIO port ID for OBC
   * @return High(True) or Low(False) of GPIO state
   */
  bool Read(const int index);
  /**
   * @fn Write
   * @brief Write the GPIO state
   * @param [in] index: element index for port_id_ vector, not the GPIO port ID for OBC
   * @param [in] is_high: High(True) or Low(False) of GPIO state
   */
  void Write(const int index, const bool is_high);

 private:
  std::vector<int> port_id_;  //!< Port ID GPIO line
  OnBoardComputer* obc_;      //!< The communication target OBC
};

#endif  // S2E_COMPONENTS_BASE_GPIO_CONNECTION_WITH_OBC_HPP_
