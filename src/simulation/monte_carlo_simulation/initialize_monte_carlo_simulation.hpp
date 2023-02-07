/**
 * @file initialize_monte_carlo_simulation.hpp
 * @brief Initialize function for Monte-Carlo Simulator
 */

#pragma once

#include "initialize_monte_carlo_parameters.hpp"
#include "monte_carlo_simulation_executor.hpp"

/**
 * @fn InitMCSim
 * @brief Initialize function for Monte-Carlo Simulator
 */
MCSimExecutor* InitMCSim(std::string file_name);
