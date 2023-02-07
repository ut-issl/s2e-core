/**
 * @file initialize_local_environment.hpp
 * @brief Initialize functions for local environment classes
 */

#ifndef S2E_ENVIRONMENT_LOCAL_INITIALIZE_LOCAL_ENVIRONMENT_H_
#define S2E_ENVIRONMENT_LOCAL_INITIALIZE_LOCAL_ENVIRONMENT_H_

#include <environment/local/MagEnvironment.h>
#include <environment/local/SRPEnvironment.h>

#include <environment/local/atmosphere.hpp>

/**
 * @fn InitMagEnvironment
 * @brief Initialize magnetic field of the earth
 * @param [in] ini_path: Path to initialize file
 */
MagEnvironment InitMagEnvironment(std::string ini_path);
/**
 * @fn InitSRPEnvironment
 * @brief Initialize solar radiation pressure
 * @param [in] ini_path: Path to initialize file
 * @param [in] local_celes_info: Local celestial information
 */
SRPEnvironment InitSRPEnvironment(std::string ini_path, LocalCelestialInformation* local_celes_info);
/**
 * @fn InitAtmosphere
 * @brief Initialize atmospheric density of the earth
 * @param [in] ini_path: Path to initialize file
 */
Atmosphere InitAtmosphere(std::string ini_path);

#endif  // S2E_ENVIRONMENT_LOCAL_INITIALIZE_LOCAL_ENVIRONMENT_H_
