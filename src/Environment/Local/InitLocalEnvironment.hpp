/**
 * @file InitLocalEnvironment.h
 * @brief Initialize functions for local environment classes
 */
#pragma once

#include <Environment/Local/Atmosphere.h>
#include <Environment/Local/MagEnvironment.h>
#include <Environment/Local/SRPEnvironment.h>

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
