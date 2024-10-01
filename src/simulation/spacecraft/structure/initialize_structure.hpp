/**
 * @file initialize_structure.hpp
 * @brief Initialize functions for spacecraft structure
 */

#ifndef S2E_SIMULATION_SPACECRAFT_STRUCTURE_INITIALIZE_STRUCTURE_HPP_
#define S2E_SIMULATION_SPACECRAFT_STRUCTURE_INITIALIZE_STRUCTURE_HPP_

#include <simulation/spacecraft/structure/structure.hpp>

namespace s2e::spacecraft {

/**
 * @fn InitKinematicsParameters
 * @brief Initialize the kinematics parameters with an ini file
 */
KinematicsParameters InitKinematicsParameters(std::string file_name);
/**
 * @fn InitSurfaces
 * @brief Initialize the multiple surfaces with an ini file
 */
std::vector<Surface> InitSurfaces(std::string file_name);
/**
 * @fn InitResidualMagneticMoment
 * @brief Initialize the RMM(Residual Magnetic Moment) parameters with an ini file
 */
ResidualMagneticMoment InitResidualMagneticMoment(std::string file_name);

}  // namespace s2e::spacecraft

#endif  // S2E_SIMULATION_SPACECRAFT_STRUCTURE_INITIALIZE_STRUCTURE_HPP_
