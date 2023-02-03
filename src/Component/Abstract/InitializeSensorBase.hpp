/**
 * @file InitializeSensorBase.hpp
 * @brief Initialize functions for SensorBase class
 */

#pragma once

#include "SensorBase.h"

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
SensorBase<N> ReadSensorBaseInformation(const std::string file_name, const double step_width_s, const std::string component_name, const std::string unit = "");

#include "InitializeSensorBase_tfs.hpp"
