/**
 * @file InitSunSensor.hpp
 * @brief Initialize functions for sun sensor
 */
#pragma once

#include <components/AOCS/SunSensor.h>

/**
 * @fn InitSunSensor
 * @brief Initialize functions for sun sensor without power port
 * @param [in] clock_gen: Clock generator
 * @param [in] sensor_id: Sensor ID
 * @param [in] fname: Path to the initialize file
 * @param [in] srp: Solar radiation pressure environment
 * @param [in] local_env: Local environment information
 */
SunSensor InitSunSensor(ClockGenerator* clock_gen, int sensor_id, const std::string fname, const SRPEnvironment* srp,
                        const LocalCelestialInformation* local_celes_info);
/**
 * @fn InitSunSensor
 * @brief Initialize functions for sun sensor with power port
 * @param [in] clock_gen: Clock generator
 * @param [in] power_port: Power port
 * @param [in] sensor_id: Sensor ID
 * @param [in] fname: Path to the initialize file
 * @param [in] srp: Solar radiation pressure environment
 * @param [in] local_env: Local environment information
 */
SunSensor InitSunSensor(ClockGenerator* clock_gen, PowerPort* power_port, int sensor_id, const std::string fname, const SRPEnvironment* srp,
                        const LocalCelestialInformation* local_celes_info);
