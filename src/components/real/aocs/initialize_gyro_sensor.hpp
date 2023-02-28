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
 * @param [in] component_step_time_s: Component step time [sec]
 * @param [in] file_name: Path to the initialize file
 * @param [in] dynamics: Dynamics information
 */
GyroSensor InitGyro(ClockGenerator* clock_generator, int sensor_id, const std::string file_name, double component_step_time_s,
                    const Dynamics* dynamics);
/**
 * @fn InitGyro
 * @brief Initialize functions for gyro sensor with power port
 * @param [in] clock_generator: Clock generator
 * @param [in] power_port: Power port
 * @param [in] sensor_id: Sensor ID
 * @param [in] component_step_time_s: Component step time [sec]
 * @param [in] file_name: Path to the initialize file
 * @param [in] dynamics: Dynamics information
 */
GyroSensor InitGyro(ClockGenerator* clock_generator, PowerPort* power_port, int sensor_id, const std::string file_name, double component_step_time_s,
                    const Dynamics* dynamics);

#endif  // S2E_COMPONENTS_REAL_AOCS_INITIALIZE_GYRO_SENSOR_HPP_
