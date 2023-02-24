/**
 * @file initialize_attitude.hpp
 * @brief Initialize function for attitude
 */

#ifndef S2E_DYNAMICS_ATTITUDE_INITIALIZE_ATTITUDE_HPP_
#define S2E_DYNAMICS_ATTITUDE_INITIALIZE_ATTITUDE_HPP_

#include "attitude.hpp"
#include "attitude_rk4.hpp"
#include "controlled_attitude.hpp"

/**
 * @fn InitAttitude
 * @brief Initialize function for Attitude
 * @param [in] file_name: Path to the initialize file
 * @param [in] orbit: Orbit information
 * @param [in] local_celestial_information: Celestial information
 * @param [in] step_width_s: Step width [sec]
 * @param [in] inertia_tensor_kgm2: Inertia tensor [kg m^2]
 * @param [in] spacecraft_id: Satellite ID
 */
Attitude* InitAttitude(std::string file_name, const Orbit* orbit, const LocalCelestialInformation* local_celestial_information,
                       const double step_width_s, const libra::Matrix<3, 3> inertia_tensor_kgm2, const int spacecraft_id);

#endif  // S2E_DYNAMICS_ATTITUDE_INITIALIZE_ATTITUDE_HPP_
