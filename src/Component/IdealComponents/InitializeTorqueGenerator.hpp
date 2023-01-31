/*
 * @file InitializeTorqueGenerator.hpp
 * @brief Initialize function for TorqueGenerator
 */
#pragma once

#include "TorqueGenerator.hpp"

/**
 * @fn InitializeTorqueGenerator
 * @brief Initialize function for TorqueGenerator
 * @param [in] clock_gen: Clock generator
 * @param [in] file_name: Path to initialize file
 * @param [in] dynamics: Dynamics information
 */
TorqueGenerator InitializeTorqueGenerator(ClockGenerator* clock_gen, const std::string file_name, const Dynamics* dynamics);