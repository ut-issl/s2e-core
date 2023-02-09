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
 * @param [in] celes_info: Celestial information
 * @param [in] ini_path: Path to initialize file
 * @param [in] stepSec: Step width [sec]
 * @param [in] current_jd: Current Julian day [day]
 * @param [in] gravity_constant: Gravity constant [m3/s2]
 * @param [in] section: Section name
 * @param [in] rel_info: Relative information
 */
Orbit* InitOrbit(const CelestialInformation* celes_info, std::string ini_path, double stepSec, double current_jd, double gravity_constant,
                 std::string section = "ORBIT", RelativeInformation* rel_info = (RelativeInformation*)nullptr);

/**
 * @fn InitializePosVel
 * @brief Initialize position and velocity depends on initialize mode
 * @param [in] ini_path: Path to initialize file
 * @param [in] current_jd: Current Julian day [day]
 * @param [in] mu_m3_s2: Gravity constant [m3/s2]
 * @param [in] section: Section name
 */
Vector<6> InitializePosVel(std::string ini_path, double current_jd, double mu_m3_s2, std::string section = "ORBIT");

#endif  // S2E_DYNAMICS_ORBIT_INITIALIZE_ORBIT_HPP_
