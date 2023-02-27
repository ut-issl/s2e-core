/**
 * @file initialize_magnetometer.hpp
 * @brief Initialize functions for magnetometer
 */

#ifndef S2E_COMPONENTS_REAL_AOCS_INITIALIZE_MAGNETOMETER_HPP_
#define S2E_COMPONENTS_REAL_AOCS_INITIALIZE_MAGNETOMETER_HPP_

#include <components/real/aocs/magnetometer.hpp>

/**
 * @fn InitMagSensor
 * @brief Initialize functions for magnetometer without power port
 * @param [in] clock_generator: Clock generator
 * @param [in] sensor_id: Sensor ID
 * @param [in] fname: Path to the initialize file
 * @param [in] compo_step_time: Component step time [sec]
 * @param [in] mgnet: Geomegnetic environment
 */
MagSensor InitMagSensor(ClockGenerator* clock_generator, int sensor_id, const std::string fname, double compo_step_time, const GeomagneticField* magnet);
/**
 * @fn InitMagSensor
 * @brief Initialize functions for magnetometer with power port
 * @param [in] clock_generator: Clock generator
 * @param [in] power_port: Power port
 * @param [in] sensor_id: Sensor ID
 * @param [in] fname: Path to the initialize file
 * @param [in] compo_step_time: Component step time [sec]
 * @param [in] mgnet: Geomegnetic environment
 */
MagSensor InitMagSensor(ClockGenerator* clock_generator, PowerPort* power_port, int sensor_id, const std::string fname, double compo_step_time,
                        const GeomagneticField* magnet);

#endif  // S2E_COMPONENTS_REAL_AOCS_INITIALIZE_MAGNETOMETER_HPP_
