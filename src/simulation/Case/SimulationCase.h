/**
 * @file SimulationCase.h
 * @brief Base class to define simulation scenario
 */

#pragma once

#include <Environment/Global/GlobalEnvironment.h>
#include <Interface/LogOutput/ILoggable.h>
#include <simulation/monte_carlo_simulation/MCSimExecutor.h>

#include "../simulation_configuration.hpp"
class Logger;

/**
 * @class SimulationCase
 * @brief Base class to define simulation scenario
 */
class SimulationCase : public ILoggable {
 public:
  /**
   * @fn SimulationCase
   * @brief Constructor
   */
  SimulationCase(std::string ini_base);
  /**
   * @fn SimulationCase
   * @brief Constructor for Monte-Carlo Simulation
   */
  SimulationCase(std::string ini_base, const MCSimExecutor& mc_sim, std::string log_path);
  /**
   * @fn ~SimulationCase
   * @brief Destructor
   */
  virtual ~SimulationCase();

  /**
   * @fn Initialize
   * @brief Virtual function to initialize the simulation scenario
   */
  virtual void Initialize() = 0;

  /**
   * @fn Main
   * @brief Virtual function of main routine of the simulation scenario
   */
  virtual void Main() = 0;

  /**
   * @fn GetLogHeader
   * @brief Virtual function of Log header settings for Monte-Carlo Simulation result
   */
  virtual std::string GetLogHeader() const = 0;
  /**
   * @fn GetLogValue
   * @brief Virtual function of Log value settings for Monte-Carlo Simulation result
   */
  virtual std::string GetLogValue() const = 0;

  // Getter
  /**
   * @fn GetSimConfig
   * @brief Return simulation setting
   */
  inline SimulationConfig& GetSimConfig() { return sim_config_; }
  /**
   * @fn GetGlobalEnvironment
   * @brief Return global environment
   */
  inline const GlobalEnvironment& GetGlobalEnvironment() const { return *glo_env_; }

 protected:
  SimulationConfig sim_config_;  //!< Simulation setting
  GlobalEnvironment* glo_env_;   //!< Global Environment
};
