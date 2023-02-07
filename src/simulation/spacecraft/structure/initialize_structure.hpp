/**
 * @file initialize_structure.hpp
 * @brief Initialize functions for spacecraft structure
 */

#ifndef S2E_SIMULATION_SPACECRAFT_STRUCTURE_INITIALIZE_STRUCTURE_H_
#define S2E_SIMULATION_SPACECRAFT_STRUCTURE_INITIALIZE_STRUCTURE_H_

#pragma once

#include <simulation/spacecraft/structure/structure.hpp>

/**
 * @fn InitKinematicsParams
 * @brief Initialize the kinematics parameters with an ini file
 */
KinematicsParams InitKinematicsParams(std::string ini_path);
/**
 * @fn InitSurfaces
 * @brief Initialize the multiple surfaces with an ini file
 */
vector<Surface> InitSurfaces(std::string ini_path);
/**
 * @fn InitRMMParams
 * @brief Initialize the RMM(Residual Magnetic Moment) parameters with an ini file
 */
RMMParams InitRMMParams(std::string ini_path);

#endif  // S2E_SIMULATION_SPACECRAFT_STRUCTURE_INITIALIZE_STRUCTURE_H_
