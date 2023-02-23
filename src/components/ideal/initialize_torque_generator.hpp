/*
 * @file initialize_torque_generator.hpp
 * @brief Initialize function for TorqueGenerator
 */

#ifndef S2E_COMPONENTS_IDEAL_INITIALIZE_TORQUE_GENERATOR_HPP_
#define S2E_COMPONENTS_IDEAL_INITIALIZE_TORQUE_GENERATOR_HPP_

#include "torque_generator.hpp"

/**
 * @fn InitializeTorqueGenerator
 * @brief Initialize function for TorqueGenerator
 * @param [in] clock_gen: Clock generator
 * @param [in] file_name: Path to initialize file
 * @param [in] dynamics: Dynamics information
 */
TorqueGenerator InitializeTorqueGenerator(ClockGenerator* clock_gen, const std::string file_name, const Dynamics* dynamics);

#endif  // S2E_COMPONENTS_IDEAL_INITIALIZE_TORQUE_GENERATOR_HPP_
