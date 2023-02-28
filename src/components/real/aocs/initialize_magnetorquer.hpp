/**
 * @file initialize_magnetorquer.hpp
 * @brief Initialize functions for magnetorquer
 */

#ifndef S2E_COMPONENTS_REAL_AOCS_INITIALIZE_MAGNETORQUER_HPP_
#define S2E_COMPONENTS_REAL_AOCS_INITIALIZE_MAGNETORQUER_HPP_

#include <components/real/aocs/magnetorquer.hpp>

/**
 * @fn InitMagnetorquer
 * @brief Initialize functions for magnetometer without power port
 * @param [in] clock_generator: Clock generator
 * @param [in] actuator_id: Actuator ID
 * @param [in] file_name: Path to the initialize file
 * @param [in] component_step_time_s: Component step time [sec]
 * @param [in] geomagnetic_field: Geomegnetic environment
 */
Magnetorquer InitMagnetorquer(ClockGenerator* clock_generator, int actuator_id, const std::string file_name, double component_step_time_s,
                              const GeomagneticField* geomagnetic_field);
/**
 * @fn InitMagnetorquer
 * @brief Initialize functions for magnetometer with power port
 * @param [in] clock_generator: Clock generator
 * @param [in] power_port: Power port
 * @param [in] actuator_id: Actuator ID
 * @param [in] file_name: Path to the initialize file
 * @param [in] component_step_time_s: Component step time [sec]
 * @param [in] geomagnetic_field: Geomegnetic environment
 */
Magnetorquer InitMagnetorquer(ClockGenerator* clock_generator, PowerPort* power_port, int actuator_id, const std::string file_name,
                              double component_step_time_s, const GeomagneticField* geomagnetic_field);

#endif  // S2E_COMPONENTS_REAL_AOCS_INITIALIZE_MAGNETORQUER_HPP_
