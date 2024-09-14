/**
 * @file monte_carlo_simulation_executor.cpp
 * @brief Monte-Carlo Simulation Executor class
 */

#include "monte_carlo_simulation_executor.hpp"

using std::string;

MonteCarloSimulationExecutor::MonteCarloSimulationExecutor(unsigned long long total_num_of_executions)
    : total_number_of_executions_(total_num_of_executions) {
  number_of_executions_done_ = 0;
  enabled_ = total_number_of_executions_ > 1 ? true : false;
  save_log_history_flag_ = !enabled_;
}

bool MonteCarloSimulationExecutor::WillExecuteNextCase() {
  if (!enabled_) {
    return (number_of_executions_done_ < 1);
  } else {
    return (number_of_executions_done_ < total_number_of_executions_);
  }
}

void MonteCarloSimulationExecutor::AtTheBeginningOfEachCase() {
  // Write CSV output of the randomization results
  ;
}

void MonteCarloSimulationExecutor::AtTheEndOfEachCase() {
  // Write CSV output of the simulation results
  number_of_executions_done_++;
}

void MonteCarloSimulationExecutor::GetInitializedMonteCarloParameterDouble(string so_name, string init_monte_carlo_parameter_name,
                                                                           double& destination) const {
  if (!enabled_) return;
  {
    string name = so_name + MonteCarloSimulationExecutor::separator_ + init_monte_carlo_parameter_name;
    if (init_parameter_list_.find(name) == init_parameter_list_.end()) {
      // Not registered in ip_list（Not defined in MCSim.ini）
      return;  // return without any update of destination
    } else {
      init_parameter_list_.at(name)->GetRandomizedScalar(destination);  // cannot use operator[] since it is const map
    }
  }
}

void MonteCarloSimulationExecutor::GetInitializedMonteCarloParameterQuaternion(string so_name, string init_monte_carlo_parameter_name,
                                                                               s2e::math::Quaternion& destination) const {
  if (!enabled_) return;
  {
    string name = so_name + MonteCarloSimulationExecutor::separator_ + init_monte_carlo_parameter_name;
    if (init_parameter_list_.find(name) == init_parameter_list_.end()) {
      // Not registered in ip_list（Not defined in MCSim.ini）
      return;  // return without any update of destination
    } else {
      init_parameter_list_.at(name)->GetRandomizedQuaternion(destination);  // cannot use operator[] since it is const map
    }
  }
}

void MonteCarloSimulationExecutor::RandomizeAllParameters() {
  for (auto ip : init_parameter_list_) {
    ip.second->Randomize();
  }
}

void MonteCarloSimulationExecutor::SetSeed(unsigned long seed, bool is_deterministic) {
  InitializedMonteCarloParameters::SetSeed(seed, is_deterministic);
}
