/**
 * @file initialize_orbit.hpp
 * @brief Initialize function for Orbit class
 */

#ifndef S2E_DYNAMICS_ORBIT_INITIALIZE_ORBIT_HPP_
#define S2E_DYNAMICS_ORBIT_INITIALIZE_ORBIT_HPP_

#include <library/orbit/orbital_elements.hpp>

#include "orbit.hpp"

class RelativeInformation;

/**
 * @fn InitOrbit
 * @brief Initialize function for Orbit class
 * @param [in] celestial_information: Celestial information
 * @param [in] initialize_file: Path to initialize file
 * @param [in] step_width_s: Step width [sec]
 * @param [in] current_time_jd: Current Julian day [day]
 * @param [in] mu_m3_s2: Gravity constant [m3/s2]
 * @param [in] section: Section name
 * @param [in] relative_information: Relative information
 */
Orbit* InitOrbit(const CelestialInformation* celestial_information, std::string initialize_file, double step_width_s, double current_time_jd,
                 double mu_m3_s2, std::string section = "ORBIT", RelativeInformation* relative_information = (RelativeInformation*)nullptr);

/**
 * @fn InitializePosVel
 * @brief Initialize position and velocity depends on initialize mode
 * @param [in] initialize_file: Path to initialize file
 * @param [in] current_time_jd: Current Julian day [day]
 * @param [in] mu_m3_s2: Gravity constant [m3/s2]
 * @param [in] section: Section name
 */
Vector<6> InitializePosVel(std::string initialize_file, double current_time_jd, double mu_m3_s2, std::string section = "ORBIT");

#endif  // S2E_DYNAMICS_ORBIT_INITIALIZE_ORBIT_HPP_
