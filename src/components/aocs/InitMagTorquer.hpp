/**
 * @file InitMagTorquer.hpp
 * @brief Initialize functions for magnetorquer
 */
#pragma once

#include <components/aocs/MagTorquer.h>

/**
 * @fn InitMagTorquer
 * @brief Initialize functions for magnetometer without power port
 * @param [in] clock_gen: Clock generator
 * @param [in] actuator_id: Actuator ID
 * @param [in] fname: Path to the initialize file
 * @param [in] compo_step_time: Component step time [sec]
 * @param [in] mag_env: Geomegnetic environment
 */
MagTorquer InitMagTorquer(ClockGenerator* clock_gen, int actuator_id, const std::string fname, double compo_step_time, const MagEnvironment* mag_env);
/**
 * @fn InitMagTorquer
 * @brief Initialize functions for magnetometer with power port
 * @param [in] clock_gen: Clock generator
 * @param [in] power_port: Power port
 * @param [in] actuator_id: Actuator ID
 * @param [in] fname: Path to the initialize file
 * @param [in] compo_step_time: Component step time [sec]
 * @param [in] mag_env: Geomegnetic environment
 */
MagTorquer InitMagTorquer(ClockGenerator* clock_gen, PowerPort* power_port, int actuator_id, const std::string fname, double compo_step_time,
                          const MagEnvironment* mag_env);
