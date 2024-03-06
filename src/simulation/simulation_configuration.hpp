/**
 * @file simulation_configuration.hpp
 * @brief Definition of struct for simulation setting information
 */

#ifndef S2E_SIMULATION_SIMULATION_CONFIGURATION_HPP_
#define S2E_SIMULATION_SIMULATION_CONFIGURATION_HPP_

#include <string>
#include <vector>

#include "../logger/logger.hpp"

/**
 * @struct SimulationConfiguration
 * @brief Simulation setting information
 */
struct SimulationConfiguration {
  std::string initialize_base_file_name_;  //!< Base file name for initialization
  Logger* main_logger_;                    //!< Main logger

  unsigned int number_of_simulated_spacecraft_;    //!< Number of simulated spacecraft
  std::vector<std::string> spacecraft_file_list_;  //!< File name list for spacecraft initialization

  unsigned int number_of_simulated_ground_station_;    //!< Number of simulated spacecraft
  std::vector<std::string> ground_station_file_list_;  //!< File name for ground station initialization

  std::string inter_sc_communication_file_;  //!< File name for inter-satellite communication initialization
  std::string gnss_file_;                    //!< File name for GNSS initialization

  /**
   * @fn ~SimulationConfiguration
   * @brief Destructor
   */
  ~SimulationConfiguration() { delete main_logger_; }
};

#endif  // S2E_SIMULATION_SIMULATION_CONFIGURATION_HPP_
