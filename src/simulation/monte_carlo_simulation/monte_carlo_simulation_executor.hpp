/**
 * @file monte_carlo_simulation_executor.hpp
 * @brief Monte-Carlo Simulation Executor class
 */

#ifndef S2E_SIMULATION_MONTE_CARLO_SIMULATION_MONTE_CARLO_SIMULATION_EXECUTOR_HPP_
#define S2E_SIMULATION_MONTE_CARLO_SIMULATION_MONTE_CARLO_SIMULATION_EXECUTOR_HPP_

#include <map>
#include <math_physics/math/vector.hpp>
#include <string>
// #include "simulation_object.hpp"
#include "initialize_monte_carlo_parameters.hpp"

namespace s2e::simulation {

/**
 * @class MonteCarloSimulationExecutor
 * @brief Monte-Carlo Simulation Executor class
 */
class MonteCarloSimulationExecutor {
 private:
  unsigned long long total_number_of_executions_;  //!< Total number of execution simulation case
  unsigned long long number_of_executions_done_;   //!< Number of executed case
  bool enabled_;                                   //!< Flag to execute Monte-Carlo Simulation or not
  bool save_log_history_flag_;                     //!< Flag to store the log for each case or not

  std::map<std::string, InitializedMonteCarloParameters*> init_parameter_list_;  //!< List of InitializedMonteCarloParameters read from MCSim.ini

 public:
  static const char separator_ = '.';  //!< Deliminator for name of SimulationObject and InitializedMonteCarloParameters in the initialization file

  /**
   * @fn MonteCarloSimulationExecutor
   * @brief Constructor
   */
  MonteCarloSimulationExecutor(unsigned long long total_num_of_executions);

  // Setter
  /**
   * @fn Enable
   * @brief Set execute flag
   */
  inline void SetEnable(bool enabled) { enabled_ = enabled; }
  /**
   * @fn SetTotalNumberOfExecutions
   * @brief Set total number of execution simulation case
   */
  inline void SetTotalNumberOfExecutions(unsigned long long number_of_executions) { total_number_of_executions_ = number_of_executions; }
  /**
   * @fn GetSaveLogHistoryFlag
   * @brief Set log history flag
   */
  inline void SetSaveLogHistoryFlag(bool set) { save_log_history_flag_ = set; }
  /**
   * @fn SetSeed
   * @brief Set seed of randomization. Use time infomation when is_deterministic = false.
   */
  static void SetSeed(unsigned long seed = 0, bool is_deterministic = false);

  // Getter
  /**
   * @fn ISEnabled
   * @brief Return execute flag
   */
  inline bool IsEnabled() const { return enabled_; }
  /**
   * @fn GetTotalNumberOfExecutions
   * @brief Return total number of execution simulation case
   */
  inline unsigned long long GetTotalNumberOfExecutions() const { return total_number_of_executions_; }
  /**
   * @fn GetNumberOfExecutionsDone
   * @brief Return number of executed case
   */
  inline unsigned long long GetNumberOfExecutionsDone() const { return number_of_executions_done_; }
  /**
   * @fn GetSaveLogHistoryFlag
   * @brief Return log history flag
   */
  inline bool GetSaveLogHistoryFlag() const {
    // Save log if MCSim is disabled or GetSaveLogHistoryFlag=ENABLED
    return (!enabled_ || save_log_history_flag_);
  }
  /**
   * @fn GetInitializedMonteCarloParameterVector
   * @brief Get randomized vector value and store it in dest_vec
   */
  template <size_t NumElement>
  void GetInitializedMonteCarloParameterVector(std::string so_name, std::string init_monte_carlo_parameter_name,
                                               math::Vector<NumElement>& destination) const;
  /**
   * @fn GetInitializedMonteCarloParameterDouble
   * @brief Get randomized value and store it in dest
   */
  void GetInitializedMonteCarloParameterDouble(std::string so_name, std::string init_monte_carlo_parameter_name, double& destination) const;
  /**
   * @fn GetInitializedMonteCarloParameterQuaternion
   * @brief Get randomized quaternion and store it in dest_quat
   */
  void GetInitializedMonteCarloParameterQuaternion(std::string so_name, std::string init_monte_carlo_parameter_name,
                                                   math::Quaternion& destination) const;

  // Calculation
  /**
   * @fn WillExecuteNextCase
   * @brief Judge execution of next simulation case
   */
  bool WillExecuteNextCase();

  /**
   * @fn AtTheBeginningOfEachCase
   * @brief Process executed before the simulation execution after the randomization in each case.
   * @details e.g. Log output of randomized results
   */
  void AtTheBeginningOfEachCase();

  /**
   * @fn AtTheEndOfEachCase
   * @brief Process executed after the each simulation case.
   * @details e.g. Log output of simulation results
   */
  void AtTheEndOfEachCase();

  template <size_t NumElement1, size_t NumElement2>
  /**
   * @fn AddInitializedMonteCarloParameter
   * @brief Add initialized parameter
   */
  void AddInitializedMonteCarloParameter(std::string so_name, std::string init_monte_carlo_parameter_name,
                                         const math::Vector<NumElement1>& mean_or_min, const math::Vector<NumElement2>& sigma_or_max,
                                         InitializedMonteCarloParameters::RandomizationType random_type);

  /**
   * @fn RandomizeAllParameters
   * @brief Randomize all initialized parameter
   */
  void RandomizeAllParameters();
};

template <size_t NumElement>
void MonteCarloSimulationExecutor::GetInitializedMonteCarloParameterVector(std::string so_name, std::string init_monte_carlo_parameter_name,
                                                                           math::Vector<NumElement>& destination) const {
  if (!enabled_) return;
  std::string name = so_name + MonteCarloSimulationExecutor::separator_ + init_monte_carlo_parameter_name;
  if (init_parameter_list_.find(name) == init_parameter_list_.end()) {
    // Not registered in ip_list（Not defined in MCSim.ini）
    return;  // return without update the destination
  } else {
    init_parameter_list_.at(name)->GetRandomizedVector(destination);  // cannot use operator[] since it is const map
  }
}

template <size_t NumElement1, size_t NumElement2>
void MonteCarloSimulationExecutor::AddInitializedMonteCarloParameter(std::string so_name, std::string init_monte_carlo_parameter_name,
                                                                     const math::Vector<NumElement1>& mean_or_min,
                                                                     const math::Vector<NumElement2>& sigma_or_max,
                                                                     InitializedMonteCarloParameters::RandomizationType random_type) {
  std::string name = so_name + MonteCarloSimulationExecutor::separator_ + init_monte_carlo_parameter_name;
  if (init_parameter_list_.find(name) == init_parameter_list_.end()) {
    // Register the parameter in ip_list if it is not registered yet
    auto newparam = new InitializedMonteCarloParameters();
    newparam->SetRandomConfiguration(mean_or_min, sigma_or_max, random_type);
    init_parameter_list_[name] = newparam;
  } else {
    // Throw error if the parameter is already registered
    throw "More than one definition of one InitializedMonteCarloParameters.";
  }
}

}  // namespace s2e::simulation

#endif  // S2E_SIMULATION_MONTE_CARLO_SIMULATION_MONTE_CARLO_SIMULATION_EXECUTOR_HPP_
