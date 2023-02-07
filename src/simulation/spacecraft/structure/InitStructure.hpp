/**
 * @file InitStructure.hpp
 * @brief Initialize functions for spacecraft structure
 */

#pragma once

#include <simulation/spacecraft/structure/Structure.h>

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
