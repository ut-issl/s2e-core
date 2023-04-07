/**
 * @file initialize_magnetometer.hpp
 * @brief Initialize functions for magnetometer
 */

#ifndef S2E_COMPONENTS_REAL_AOCS_INITIALIZE_MAGNETOMETER_HPP_
#define S2E_COMPONENTS_REAL_AOCS_INITIALIZE_MAGNETOMETER_HPP_

#include <components/real/aocs/magnetometer.hpp>

/**
 * @fn InitMagetometer
 * @brief Initialize functions for magnetometer without power port
 * @param [in] clock_generator: Clock generator
 * @param [in] sensor_id: Sensor ID
 * @param [in] file_name: Path to the initialize file
 * @param [in] component_step_time_s: Component step time [sec]
 * @param [in] geomagnetic_field: Geomegnetic environment
 */
Magnetometer InitMagetometer(ClockGenerator* clock_generator, int sensor_id, const std::string file_name, double component_step_time_s,
                             const GeomagneticField* geomagnetic_field);
/**
 * @fn InitMagetometer
 * @brief Initialize functions for magnetometer with power port
 * @param [in] clock_generator: Clock generator
 * @param [in] power_port: Power port
 * @param [in] sensor_id: Sensor ID
 * @param [in] file_name: Path to the initialize file
 * @param [in] component_step_time_s: Component step time [sec]
 * @param [in] geomagnetic_field: Geomegnetic environment
 */
Magnetometer InitMagetometer(ClockGenerator* clock_generator, PowerPort* power_port, int sensor_id, const std::string file_name,
                             double component_step_time_s, const GeomagneticField* geomagnetic_field);

#endif  // S2E_COMPONENTS_REAL_AOCS_INITIALIZE_MAGNETOMETER_HPP_
