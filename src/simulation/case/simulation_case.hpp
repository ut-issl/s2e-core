/**
 * @file simulation_case.hpp
 * @brief Base class to define simulation scenario
 */

#ifndef S2E_SIMULATION_CASE_SIMULATION_CASE_HPP_
#define S2E_SIMULATION_CASE_SIMULATION_CASE_HPP_

#include <environment/global/global_environment.hpp>
#include <logger/loggable.hpp>
#include <simulation/monte_carlo_simulation/monte_carlo_simulation_executor.hpp>

#include "../simulation_configuration.hpp"
class Logger;

namespace s2e::simulation {

/**
 * @class SimulationCase
 * @brief Base class to define simulation scenario
 */
class SimulationCase : public logger::ILoggable {
 public:
  /**
   * @fn SimulationCase
   * @brief Constructor
   * @param[in] initialize_base_file: File path to initialize base file
   */
  SimulationCase(const std::string initialize_base_file);
  /**
   * @fn SimulationCase
   * @brief Constructor for Monte-Carlo Simulation
   * @param[in] initialize_base_file: File path to initialize base file
   * @param[in] monte_carlo_simulator: Monte-Carlo simulator
   * @param[in] log_path: Log output file path for Monte-Carlo simulation
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
  virtual void Initialize();

  /**
   * @fn Main
   * @brief Virtual function of main routine of the simulation scenario
   */
  virtual void Main();

  /**
   * @fn GetLogHeader
   * @brief Virtual function of Log header settings for Monte-Carlo Simulation result
   */
  virtual std::string GetLogHeader() const;
  /**
   * @fn GetLogValue
   * @brief Virtual function of Log value settings for Monte-Carlo Simulation result
   */
  virtual std::string GetLogValue() const;

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
  inline const environment::GlobalEnvironment& GetGlobalEnvironment() const { return *global_environment_; }

 protected:
  SimulationConfiguration simulation_configuration_;    //!< Simulation setting
  environment::GlobalEnvironment* global_environment_;  //!< Global Environment

  /**
   * @fn InitializeSimulationConfiguration
   * @brief Initialize simulation configuration
   * @param[in] initialize_base_file: File path to initialize base file
   */
  void InitializeSimulationConfiguration(const std::string initialize_base_file);

  /**
   * @fn InitializeTargetObjects
   * @brief Virtual function to initialize target objects(spacecraft and ground station) for the simulation
   */
  virtual void InitializeTargetObjects() = 0;

  /**
   * @fn UpdateTargetObjects
   * @brief Virtual function to update target objects(spacecraft and ground station)
   */
  virtual void UpdateTargetObjects() = 0;
};

}  // namespace s2e::simulation

#endif  // S2E_SIMULATION_CASE_SIMULATION_CASE_HPP_
