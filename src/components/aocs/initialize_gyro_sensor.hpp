/**
 * @file initialize_gyro_sensor.hpp
 * @brief Initialize functions for gyro sensor
 */

#ifndef S2E_COMPONENTS_AOCS_INITIALIZE_GYRO_SENSOR_H_
#define S2E_COMPONENTS_AOCS_INITIALIZE_GYRO_SENSOR_H_

#include <components/aocs/gyro_sensor.hpp>

/**
 * @fn InitGyro
 * @brief Initialize functions for gyro sensor without power port
 * @param [in] clock_gen: Clock generator
 * @param [in] sensor_id: Sensor ID
 * @param [in] compo_step_time: Component step time [sec]
 * @param [in] fname: Path to the initialize file
 * @param [in] dynamics: Dynamics information
 */
Gyro InitGyro(ClockGenerator* clock_gen, int sensor_id, const std::string fname, double compo_step_time, const Dynamics* dynamics);
/**
 * @fn InitGyro
 * @brief Initialize functions for gyro sensor with power port
 * @param [in] clock_gen: Clock generator
 * @param [in] power_port: Power port
 * @param [in] sensor_id: Sensor ID
 * @param [in] compo_step_time: Component step time [sec]
 * @param [in] fname: Path to the initialize file
 * @param [in] dynamics: Dynamics information
 */
Gyro InitGyro(ClockGenerator* clock_gen, PowerPort* power_port, int sensor_id, const std::string fname, double compo_step_time,
              const Dynamics* dynamics);

#endif  // S2E_COMPONENTS_AOCS_INITIALIZE_GYRO_SENSOR_H_