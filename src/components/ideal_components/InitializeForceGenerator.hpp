/*
 * @file InitializeForceGenerator.hpp
 * @brief Initialize function for ForceGenerator
 */
#pragma once

#include "force_generator.hpp"

/**
 * @fn InitializeForceGenerator
 * @brief Initialize function for ForceGenerator
 * @param [in] clock_gen: Clock generator
 * @param [in] file_name: Path to initialize file
 * @param [in] dynamics: Dynamics information
 */
ForceGenerator InitializeForceGenerator(ClockGenerator* clock_gen, const std::string file_name, const Dynamics* dynamics);
