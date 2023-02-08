/**
 * @file initialize_sensor_base.hpp
 * @brief Initialize functions for SensorBase class
 */

#ifndef S2E_COMPONENTS_BASE_CLASSES_INITIALIZE_SENSOR_BASE_H_
#define S2E_COMPONENTS_BASE_CLASSES_INITIALIZE_SENSOR_BASE_H_

#include "sensor_base.hpp"

/**
 * @fn ReadSensorBaseInformation
 * @brief Read information from initialize file for SensorBase class
 * @note It is recommended to use this function for all sensors inherits the SensorBase class
 * @param [in] file_name: Path to the initialize file
 * @param [in] step_width_s: Step width of component update [sec]
 * @param [in] component_name: Component name
 * @param [in] unit: Unit of the sensor
 */
template <size_t N>
SensorBase<N> ReadSensorBaseInformation(const std::string file_name, const double step_width_s, const std::string component_name,
                                        const std::string unit = "");

#include "initialize_sensor_base_tfs.hpp"

#endif  // S2E_COMPONENTS_BASE_CLASSES_INITIALIZE_SENSOR_BASE_H_
