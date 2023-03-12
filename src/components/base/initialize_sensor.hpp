/**
 * @file initialize_sensor.hpp
 * @brief Initialize functions for Sensor class
 */

#ifndef S2E_COMPONENTS_BASE_INITIALIZE_SENSOR_HPP_
#define S2E_COMPONENTS_BASE_INITIALIZE_SENSOR_HPP_

#include "sensor.hpp"

/**
 * @fn ReadSensorInformation
 * @brief Read information from initialize file for Sensor class
 * @note It is recommended to use this function for all sensors inherits the Sensor class
 * @param [in] file_name: Path to the initialize file
 * @param [in] step_width_s: Step width of component update [sec]
 * @param [in] component_name: Component name
 * @param [in] unit: Unit of the sensor
 */
template <size_t N>
Sensor<N> ReadSensorInformation(const std::string file_name, const double step_width_s, const std::string component_name,
                                const std::string unit = "");

#include "initialize_sensor_template_functions.hpp"

#endif  // S2E_COMPONENTS_BASE_INITIALIZE_SENSOR_HPP_
