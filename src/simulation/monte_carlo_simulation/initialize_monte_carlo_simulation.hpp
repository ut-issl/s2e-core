/**
 * @file initialize_monte_carlo_simulation.hpp
 * @brief Initialize function for Monte-Carlo Simulator
 */

#ifndef S2E_SIMULATION_MONTE_CARLO_SIMULATION_INITIALIZE_MONTE_CARLO_SIMULATION_H_
#define S2E_SIMULATION_MONTE_CARLO_SIMULATION_INITIALIZE_MONTE_CARLO_SIMULATION_H_

#include "initialize_monte_carlo_parameters.hpp"
#include "monte_carlo_simulation_executor.hpp"

/**
 * @fn InitMCSim
 * @brief Initialize function for Monte-Carlo Simulator
 */
MCSimExecutor* InitMCSim(std::string file_name);

#endif  // S2E_SIMULATION_MONTE_CARLO_SIMULATION_INITIALIZE_MONTE_CARLO_SIMULATION_H_
