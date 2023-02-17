/**
 * @file initialize_disturbances.hpp
 * @brief Define initialize functions for disturbances
 */

#ifndef S2E_DISTURBANCES_INITIALIZE_DISTURBANCES_HPP_
#define S2E_DISTURBANCES_INITIALIZE_DISTURBANCES_HPP_

#include <disturbances/air_drag.hpp>
#include <disturbances/geopotential.hpp>
#include <disturbances/gravity_gradient.hpp>
#include <disturbances/magnetic_disturbance.hpp>
#include <disturbances/solar_radiation_pressure_disturbance.hpp>
#include <disturbances/third_body_gravity.hpp>

/**
 * @fn InitAirDrag
 * @brief Initialize AirDrag class
 * @param [in] initialize_file_path: Initialize file path
 * @param [in] surfaces: surface information of the spacecraft
 * @param [in] center_of_gravity_b_m: Center of gravity position vector at body frame [m]
 */
AirDrag InitAirDrag(const std::string initialize_file_path, const std::vector<Surface>& surfaces, const Vector<3>& center_of_gravity_b_m);

/**
 * @fn InitSolarRadiationPressureDisturbance
 * @brief Initialize SolarRadiation class
 * @param [in] initialize_file_path: Initialize file path
 * @param [in] surfaces: surface information of the spacecraft
 * @param [in] center_of_gravity_b_m: Center of gravity position vector at body frame [m]
 */
SolarRadiation InitSolarRadiationPressureDisturbance(const std::string initialize_file_path, const std::vector<Surface>& surfaces,
                                                     const Vector<3>& center_of_gravity_b_m);

/**
 * @fn InitGravityGradient
 * @brief Initialize GravityGradient class with earth gravitational constant
 * @param [in] initialize_file_path: Initialize file path
 */
GravityGradient InitGravityGradient(const std::string initialize_file_path);

/**
 * @fn InitGravityGradient
 * @brief Initialize GravityGradient class with earth gravitational constant
 * @param [in] initialize_file_path: Initialize file path
 * @param [in] mu_m3_s2: Gravitational constant [m3/s2]
 */
GravityGradient InitGravityGradient(const std::string initialize_file_path, const double mu_m3_s2);

/**
 * @fn InitMagneticDisturbance
 * @brief Initialize MagDisturbance class with earth gravitational constant
 * @param [in] initialize_file_path: Initialize file path
 * @param [in] rmm_params: RMM parameters
 */
MagDisturbance InitMagneticDisturbance(const std::string initialize_file_path, const RMMParams& rmm_params);

/**
 * @fn InitGeoPotential
 * @brief Initialize GeoPotential class with earth gravitational constant
 * @param [in] initialize_file_path: Initialize file path
 */
GeoPotential InitGeoPotential(const std::string initialize_file_path);

/**
 * @fn InitThirdBodyGravity
 * @brief Initialize ThirdBodyGravity class with earth gravitational constant
 * @param [in] initialize_file_path: Initialize file path
 * @param [in] ini_path_celes: Initialize file path for the celestial information
 */
ThirdBodyGravity InitThirdBodyGravity(const std::string initialize_file_path, const std::string ini_path_celes);

#endif  // S2E_DISTURBANCES_INITIALIZE_DISTURBANCES_HPP_
