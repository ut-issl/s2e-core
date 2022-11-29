/**
 * @file SimulationConfig.h
 * @brief Definition of struct for simulation setting information
 */

#pragma once
#include <string>
#include <vector>

#include "../Interface/LogOutput/Logger.h"

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
   * @brief Deconstructor
   */
  ~SimulationConfig() { delete main_logger_; }
};
