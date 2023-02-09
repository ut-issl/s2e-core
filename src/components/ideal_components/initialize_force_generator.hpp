/*
 * @file initialize_force_generator.hpp
 * @brief Initialize function for ForceGenerator
 */

#ifndef S2E_COMPONENTS_IDEAL_COMPONENTS_INITIALIZE_FORCE_GENERATOR_HPP_
#define S2E_COMPONENTS_IDEAL_COMPONENTS_INITIALIZE_FORCE_GENERATOR_HPP_

#include "force_generator.hpp"

/**
 * @fn InitializeForceGenerator
 * @brief Initialize function for ForceGenerator
 * @param [in] clock_gen: Clock generator
 * @param [in] file_name: Path to initialize file
 * @param [in] dynamics: Dynamics information
 */
ForceGenerator InitializeForceGenerator(ClockGenerator* clock_gen, const std::string file_name, const Dynamics* dynamics);

#endif  // S2E_COMPONENTS_IDEAL_COMPONENTS_INITIALIZE_FORCE_GENERATOR_HPP_
