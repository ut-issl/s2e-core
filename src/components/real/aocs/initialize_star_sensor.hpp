/**
 * @file initialize_star_sensor.hpp
 * @brief Initialize functions for star sensor
 */

#ifndef S2E_COMPONENTS_REAL_AOCS_INITIALIZE_STAR_SENSOR_HPP_
#define S2E_COMPONENTS_REAL_AOCS_INITIALIZE_STAR_SENSOR_HPP_

#include <components/real/aocs/star_sensor.hpp>

/**
 * @fn InitSTT
 * @brief Initialize functions for StarSensor without power port
 * @param [in] clock_generator: Clock generator
 * @param [in] sensor_id: Sensor ID
 * @param [in] fname: Path to the initialize file
 * @param [in] compo_step_time: Component step time [sec]
 * @param [in] dynamics: Dynamics information
 * @param [in] local_environment: Local environment information
 */
StarSensor InitSTT(ClockGenerator* clock_generator, int sensor_id, const std::string fname, double compo_step_time, const Dynamics* dynamics,
                   const LocalEnvironment* local_environment);
/**
 * @fn InitSTT
 * @brief Initialize functions for StarSensor with power port
 * @param [in] clock_generator: Clock generator
 * @param [in] power_port: Power port
 * @param [in] sensor_id: Sensor ID
 * @param [in] fname: Path to the initialize file
 * @param [in] compo_step_time: Component step time [sec]
 * @param [in] dynamics: Dynamics information
 * @param [in] local_environment: Local environment information
 */
StarSensor InitSTT(ClockGenerator* clock_generator, PowerPort* power_port, int sensor_id, const std::string fname, double compo_step_time,
                   const Dynamics* dynamics, const LocalEnvironment* local_environment);

#endif  // S2E_COMPONENTS_REAL_AOCS_INITIALIZE_STAR_SENSOR_HPP_
