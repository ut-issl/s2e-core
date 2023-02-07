/**
 * @file InitMcSim.hpp
 * @brief Initialize function for Monte-Carlo Simulator
 */

#pragma once

#include "InitParameter.h"
#include "monte_carlo_simulation_executor.hpp"

/**
 * @fn InitMCSim
 * @brief Initialize function for Monte-Carlo Simulator
 */
MCSimExecutor* InitMCSim(std::string file_name);
