/**
 * @file monte_carlo_simulation_executor.cpp
 * @brief Monte-Carlo Simulation Executor class
 */

#include "monte_carlo_simulation_executor.hpp"

using std::string;

MCSimExecutor::MCSimExecutor(unsigned long long total_num_of_executions) : total_num_of_executions_(total_num_of_executions) {
  num_of_executions_done_ = 0;
  enabled_ = total_num_of_executions_ > 1 ? true : false;
  log_history_ = !enabled_;
}

bool MCSimExecutor::WillExecuteNextCase() {
  if (!enabled_) {
    return (num_of_executions_done_ < 1);
  } else {
    return (num_of_executions_done_ < total_num_of_executions_);
  }
}

void MCSimExecutor::AtTheBeginningOfEachCase() {
  // Write CSV output of the randomization results
  ;
}

void MCSimExecutor::AtTheEndOfEachCase() {
  // Write CSV output of the simulation results
  num_of_executions_done_++;
}

void MCSimExecutor::GetInitParameterDouble(string so_name, string ip_name, double& destination) const {
  if (!enabled_) return;
  {
    string name = so_name + MCSimExecutor::separator_ + ip_name;
    if (ip_list_.find(name) == ip_list_.end()) {
      // Not registered in ip_list（Not defined in MCSim.ini）
      return;  // return without any update of destination
    } else {
      ip_list_.at(name)->GetDouble(destination);  // cannot use operator[] since it is const map
    }
  }
}

void MCSimExecutor::GetInitParameterQuaternion(string so_name, string ip_name, libra::Quaternion& destination) const {
  if (!enabled_) return;
  {
    string name = so_name + MCSimExecutor::separator_ + ip_name;
    if (ip_list_.find(name) == ip_list_.end()) {
      // Not registered in ip_list（Not defined in MCSim.ini）
      return;  // return without any update of destination
    } else {
      ip_list_.at(name)->GetQuaternion(destination);  // cannot use operator[] since it is const map
    }
  }
}

void MCSimExecutor::RandomizeAllParameters() {
  for (auto ip : ip_list_) {
    ip.second->Randomize();
  }
}

void MCSimExecutor::SetSeed(unsigned long seed, bool is_deterministic) { InitParameter::SetSeed(seed, is_deterministic); }
