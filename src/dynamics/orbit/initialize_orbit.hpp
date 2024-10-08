/**
 * @file initialize_orbit.hpp
 * @brief Initialize function for Orbit class
 */

#ifndef S2E_DYNAMICS_ORBIT_INITIALIZE_ORBIT_HPP_
#define S2E_DYNAMICS_ORBIT_INITIALIZE_ORBIT_HPP_

#include <math_physics/orbit/orbital_elements.hpp>

#include "orbit.hpp"

namespace s2e::simulation {
class RelativeInformation;
}

namespace s2e::dynamics::orbit {

/**
 * @fn InitOrbit
 * @brief Initialize function for Orbit class
 * @param [in] celestial_information: Celestial information
 * @param [in] initialize_file: Path to initialize file
 * @param [in] step_width_s: Step width [sec]
 * @param [in] current_time_jd: Current Julian day [day]
 * @param [in] gravity_constant_m3_s2: Gravity constant [m3/s2]
 * @param [in] section: Section name
 * @param [in] relative_information: Relative information
 */
Orbit* InitOrbit(const environment::CelestialInformation* celestial_information, std::string initialize_file, double step_width_s,
                 double current_time_jd, double gravity_constant_m3_s2, std::string section = "ORBIT",
                 simulation::RelativeInformation* relative_information = (simulation::RelativeInformation*)nullptr);

/**
 * @fn InitializePosVel
 * @brief Initialize position and velocity depends on initialize mode
 * @param [in] initialize_file: Path to initialize file
 * @param [in] current_time_jd: Current Julian day [day]
 * @param [in] gravity_constant_m3_s2: Gravity constant [m3/s2]
 * @param [in] section: Section name
 */
math::Vector<6> InitializePosVel(std::string initialize_file, double current_time_jd, double gravity_constant_m3_s2, std::string section = "ORBIT");

}  // namespace s2e::dynamics::orbit

#endif  // S2E_DYNAMICS_ORBIT_INITIALIZE_ORBIT_HPP_
