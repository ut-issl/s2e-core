/**
 * @file simulation_configuration.hpp
 * @brief Definition of struct for simulation setting information
 */

#ifndef S2E_SIMULATION_SIMULATION_CONFIGURATION_HPP_
#define S2E_SIMULATION_SIMULATION_CONFIGURATION_HPP_

#include <string>
#include <vector>

#include "../library/logger/logger.hpp"

/**
 * @struct SimulationConfig
 * @brief Simulation setting information
 */
struct SimulationConfig {
  std::string ini_base_fname_;         //!< Base file name for initialization
  Logger* main_logger_;                //!< Main logger
  int num_of_simulated_spacecraft_;    //!< Number of simulated spacecraft
  std::vector<std::string> sat_file_;  //!< File name list for spacecraft initialization
  std::string gs_file_;                //!< File name for ground station initialization
  std::string inter_sat_comm_file_;    //!< File name for inter-satellite communication initialization
  std::string gnss_file_;              //!< File name for GNSS initialization

  /**
   * @fn ~SimulationConfig
   * @brief Destructor
   */
  ~SimulationConfig() { delete main_logger_; }
};

#endif  // S2E_SIMULATION_SIMULATION_CONFIGURATION_HPP_
