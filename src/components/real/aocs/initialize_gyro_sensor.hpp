/**
 * @file initialize_gyro_sensor.hpp
 * @brief Initialize functions for gyro sensor
 */

#ifndef S2E_COMPONENTS_REAL_AOCS_INITIALIZE_GYRO_SENSOR_HPP_
#define S2E_COMPONENTS_REAL_AOCS_INITIALIZE_GYRO_SENSOR_HPP_

#include <components/real/aocs/gyro_sensor.hpp>

/**
 * @fn InitGyro
 * @brief Initialize functions for gyro sensor without power port
 * @param [in] clock_generator: Clock generator
 * @param [in] sensor_id: Sensor ID
 * @param [in] compo_step_time: Component step time [sec]
 * @param [in] fname: Path to the initialize file
 * @param [in] dynamics: Dynamics information
 */
GyroSensor InitGyro(ClockGenerator* clock_generator, int sensor_id, const std::string fname, double compo_step_time, const Dynamics* dynamics);
/**
 * @fn InitGyro
 * @brief Initialize functions for gyro sensor with power port
 * @param [in] clock_generator: Clock generator
 * @param [in] power_port: Power port
 * @param [in] sensor_id: Sensor ID
 * @param [in] compo_step_time: Component step time [sec]
 * @param [in] fname: Path to the initialize file
 * @param [in] dynamics: Dynamics information
 */
GyroSensor InitGyro(ClockGenerator* clock_generator, PowerPort* power_port, int sensor_id, const std::string fname, double compo_step_time,
                    const Dynamics* dynamics);

#endif  // S2E_COMPONENTS_REAL_AOCS_INITIALIZE_GYRO_SENSOR_HPP_
