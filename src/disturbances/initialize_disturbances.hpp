/**
 * @file initialize_disturbances.hpp
 * @brief Define initialize functions for disturbances
 */

#ifndef S2E_DISTURBANCES_INITIALIZE_DISTURBANCES_HPP_
#define S2E_DISTURBANCES_INITIALIZE_DISTURBANCES_HPP_

#include <disturbances/air_drag.hpp>
#include <disturbances/geopotential.hpp>
#include <disturbances/gravity_gradient.hpp>
#include <disturbances/lunar_gravity_field.hpp>
#include <disturbances/magnetic_disturbance.hpp>
#include <disturbances/solar_radiation_pressure_disturbance.hpp>
#include <disturbances/third_body_gravity.hpp>

/**
 * @fn InitGeopotential
 * @brief Initialize Geopotential class with earth gravitational constant
 * @param [in] initialize_file_path: Initialize file path
 */
Geopotential InitGeopotential(const std::string initialize_file_path);

/**
 * @fn InitLunarGravityField
 * @brief Initialize LunarGravityField class with earth gravitational constant
 * @param [in] initialize_file_path: Initialize file path
 */
LunarGravityField InitLunarGravityField(const std::string initialize_file_path);

/**
 * @fn InitThirdBodyGravity
 * @brief Initialize ThirdBodyGravity class with earth gravitational constant
 * @param [in] initialize_file_path: Initialize file path
 * @param [in] ini_path_celes: Initialize file path for the celestial information
 */
ThirdBodyGravity InitThirdBodyGravity(const std::string initialize_file_path, const std::string ini_path_celes);

#endif  // S2E_DISTURBANCES_INITIALIZE_DISTURBANCES_HPP_
