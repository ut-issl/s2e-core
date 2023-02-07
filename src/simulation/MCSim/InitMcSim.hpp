/**
 * @file InitMcSim.hpp
 * @brief Initialize function for Monte-Carlo Simulator
 */

#pragma once

#include "InitParameter.h"
#include "MCSimExecutor.h"

/**
 * @fn InitMCSim
 * @brief Initialize function for Monte-Carlo Simulator
 */
MCSimExecutor* InitMCSim(std::string file_name);
