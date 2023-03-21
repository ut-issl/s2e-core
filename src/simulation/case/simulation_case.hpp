/**
 * @file simulation_case.hpp
 * @brief Base class to define simulation scenario
 */

#ifndef S2E_SIMULATION_CASE_SIMULATION_CASE_HPP_
#define S2E_SIMULATION_CASE_SIMULATION_CASE_HPP_

#include <environment/global/global_environment.hpp>
#include <library/logger/loggable.hpp>
#include <simulation/monte_carlo_simulation/monte_carlo_simulation_executor.hpp>

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
  SimulationCase(const std::string initialize_base_file);
  /**
   * @fn SimulationCase
   * @brief Constructor for Monte-Carlo Simulation
   */
  SimulationCase(const std::string initialize_base_file, const MonteCarloSimulationExecutor& monte_carlo_simulator, const std::string log_path);
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
   * @fn GetSimulationConfiguration
   * @brief Return simulation setting
   */
  inline SimulationConfiguration& GetSimulationConfiguration() { return simulation_configuration_; }
  /**
   * @fn GetGlobalEnvironment
   * @brief Return global environment
   */
  inline const GlobalEnvironment& GetGlobalEnvironment() const { return *global_environment_; }

 protected:
  SimulationConfiguration simulation_configuration_;  //!< Simulation setting
  GlobalEnvironment* global_environment_;             //!< Global Environment

  /**
   * @fn GetGlobalEnvironment
   * @brief Return global environment
   * @param[in] initialize_base_file: File path to initialize base file
   */
  void InitializeSimulationConfiguration(const std::string initialize_base_file);
};

#endif  // S2E_SIMULATION_CASE_SIMULATION_CASE_HPP_
