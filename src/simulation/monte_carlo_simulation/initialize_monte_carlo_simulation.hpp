/**
 * @file initialize_monte_carlo_simulation.hpp
 * @brief Initialize function for Monte-Carlo Simulator
 */

#ifndef S2E_SIMULATION_MONTE_CARLO_SIMULATION_INITIALIZE_MONTE_CARLO_SIMULATION_HPP_
#define S2E_SIMULATION_MONTE_CARLO_SIMULATION_INITIALIZE_MONTE_CARLO_SIMULATION_HPP_

#include "initialize_monte_carlo_parameters.hpp"
#include "monte_carlo_simulation_executor.hpp"

namespace s2e::simulation {

/**
 * @fn InitMonteCarloSimulation
 * @brief Initialize function for Monte-Carlo Simulator
 */
MonteCarloSimulationExecutor* InitMonteCarloSimulation(std::string file_name);

} // namespace s2e::simulation

#endif  // S2E_SIMULATION_MONTE_CARLO_SIMULATION_INITIALIZE_MONTE_CARLO_SIMULATION_HPP_
