/**
 * @file InitMagSensor.hpp
 * @brief Initialize functions for magnetometer
 */
#pragma once

#include <components/AOCS/MagSensor.h>

/**
 * @fn InitMagSensor
 * @brief Initialize functions for magnetometer without power port
 * @param [in] clock_gen: Clock generator
 * @param [in] sensor_id: Sensor ID
 * @param [in] fname: Path to the initialize file
 * @param [in] compo_step_time: Component step time [sec]
 * @param [in] mgnet: Geomegnetic environment
 */
MagSensor InitMagSensor(ClockGenerator* clock_gen, int sensor_id, const std::string fname, double compo_step_time, const MagEnvironment* magnet);
/**
 * @fn InitMagSensor
 * @brief Initialize functions for magnetometer with power port
 * @param [in] clock_gen: Clock generator
 * @param [in] power_port: Power port
 * @param [in] sensor_id: Sensor ID
 * @param [in] fname: Path to the initialize file
 * @param [in] compo_step_time: Component step time [sec]
 * @param [in] mgnet: Geomegnetic environment
 */
MagSensor InitMagSensor(ClockGenerator* clock_gen, PowerPort* power_port, int sensor_id, const std::string fname, double compo_step_time,
                        const MagEnvironment* magnet);
