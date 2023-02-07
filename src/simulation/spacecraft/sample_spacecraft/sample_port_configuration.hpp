/**
 * @file sample_port_configuration.hpp
 * @brief An example of port configuration management
 */

#pragma once

/**
 * @enum PowerPortConfig
 * @brief ID list of electrical power switch ports
 * @details Register sequential same with port_id. Use NONE_1, NONE_2 if the number is skipped.
 */
enum PowerPortConfig {
  OBC_BUS,
  GYRO_5V,
  COMPONENT_MAX  //!< Maximum port number. Do not remove. Place on the bottom.
};

/**
 * @enum UARTPortConfig
 * @brief ID list of serial communication ports with UART
 * @details Register sequential same with port_id. Use NONE_1, NONE_2 if the number is skipped.
 */
enum UARTPortConfig {
  GYRO = 0,
  UART_COMPONENT_MAX  //!< Maximum port number. Do not remove. Place on the bottom.
};
