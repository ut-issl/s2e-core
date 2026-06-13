/**
 * @file sample_port_configuration.hpp
 * @brief An example of port configuration management
 */

#ifndef S2E_SIMULATION_SAMPLE_SPACECRAFT_SAMPLE_PORT_CONFIGURATION_HPP_
#define S2E_SIMULATION_SAMPLE_SPACECRAFT_SAMPLE_PORT_CONFIGURATION_HPP_

namespace s2e::sample {

/**
 * @enum PowerPortConfig
 * @brief ID list of electrical power switch ports
 * @details Register sequential same with port_id. Use kNone1, kNone2 if the number is skipped.
 */
enum class PowerPortConfig {
  kObcBus,
  kGyro5v,
  kComponentMax  //!< Maximum port number. Do not remove. Place on the bottom.
};

/**
 * @enum UARTPortConfig
 * @brief ID list of serial communication ports with UART
 * @details Register sequential same with port_id. Use kNone1, kNone2 if the number is skipped.
 */
enum class UARTPortConfig {
  kGyro = 0,
  kUartComponentMax  //!< Maximum port number. Do not remove. Place on the bottom.
};

}  // namespace s2e::sample

#endif  // S2E_SIMULATION_SAMPLE_SPACECRAFT_SAMPLE_PORT_CONFIGURATION_HPP_
