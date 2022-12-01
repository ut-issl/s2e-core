/**
 * @file MCSimExecutor.h
 * @brief Monte-Carlo Simulation Executor class
 */

#pragma once

#include <Library/math/Vector.hpp>
#include <map>
#include <string>
//#include "SimulationObject.h"
#include "InitParameter.h"

using libra::Vector;

/**
 * @class MCSimExecutor
 * @brief Monte-Carlo Simulation Executor class
 */
class MCSimExecutor {
 private:
  unsigned long long total_num_of_executions_;  //!< Total number of execution simulation case
  unsigned long long num_of_executions_done_;   //!< Number of executed case
  bool enabled_;                                //!< Flag to execute Monte-Carlo Simulation or not
  bool log_history_;                            //!< Flag to store the log for each case or not

  std::map<std::string, InitParameter*> ip_list_;  //!< List of InitParameters read from MCSim.ini

 public:
  static const char separator_ = '.';  //!< Deliminator for name of SimulationObject and InitParameter in the initialization file

  /**
   * @fn MCSimExecutor
   * @brief Constructor
   */
  MCSimExecutor(unsigned long long total_num_of_executions);

  // Setter
  /**
   * @fn Enable
   * @brief Set execute flag
   */
  inline void Enable(bool enabled);
  /**
   * @fn SetTotalNumOfExecutions
   * @brief Set total number of execution simulation case
   */
  inline void SetTotalNumOfExecutions(unsigned long long num_of_executions);
  /**
   * @fn LogHistory
   * @brief Set log history flag
   */
  inline void LogHistory(bool set);
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
  inline bool IsEnabled() const;
  /**
   * @fn GetTotalNumOfExecutions
   * @brief Return total number of execution simulation case
   */
  inline unsigned long long GetTotalNumOfExecutions() const;
  /**
   * @fn GetNumOfExecutionsDone
   * @brief Return number of executed case
   */
  inline unsigned long long GetNumOfExecutionsDone() const;
  /**
   * @fn LogHistory
   * @brief Return log history flag
   */
  inline bool LogHistory() const;
  /**
   * @fn GetInitParameterVec
   * @brief Get randomized vector value and store it in dest_vec
   */
  template <size_t NumElement>
  void GetInitParameterVec(std::string so_name, std::string ip_name, Vector<NumElement>& dst_vec) const;
  /**
   * @fn GetInitParameterDouble
   * @brief Get randomized value and store it in dest
   */
  void GetInitParameterDouble(std::string so_name, std::string ip_name, double& dst) const;
  /**
   * @fn GetInitParameterQuaternion
   * @brief Get randomized quaternion and store it in dest_quat
   */
  void GetInitParameterQuaternion(std::string so_name, std::string ip_name, Quaternion& dst_quat) const;

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
   * @fn AddInitParameter
   * @brief Add initialized parameter
   */
  void AddInitParameter(std::string so_name, std::string ip_name, const Vector<NumElement1>& mean_or_min, const Vector<NumElement2>& sigma_or_max,
                        InitParameter::RandomizationType rnd_type);

  /**
   * @fn RandomizeAllParameters
   * @brief Randomize all initialized parameter
   */
  void RandomizeAllParameters();
};

void MCSimExecutor::Enable(bool enabled) { enabled_ = enabled; }

bool MCSimExecutor::IsEnabled() const { return enabled_; }

void MCSimExecutor::SetTotalNumOfExecutions(unsigned long long num_of_executions) { total_num_of_executions_ = num_of_executions; }

unsigned long long MCSimExecutor::GetTotalNumOfExecutions() const { return total_num_of_executions_; }

inline unsigned long long MCSimExecutor::GetNumOfExecutionsDone() const { return num_of_executions_done_; }

bool MCSimExecutor::LogHistory() const {
  // Save log if MCSim is disabled or LogHistory=ENABLED
  return (!enabled_ || log_history_);
}

void MCSimExecutor::LogHistory(bool set) { log_history_ = set; }

template <size_t NumElement>
void MCSimExecutor::GetInitParameterVec(std::string so_name, std::string ip_name, Vector<NumElement>& dst_vec) const {
  if (!enabled_) return;
  std::string name = so_name + MCSimExecutor::separator_ + ip_name;
  if (ip_list_.find(name) == ip_list_.end()) {
    // Not registered in ip_list（Not defined in MCSim.ini）
    return;  // return without update the dst_vec
  } else {
    ip_list_.at(name)->GetVec(dst_vec);  // cannot use operator[] since it is const map
  }
}

template <size_t NumElement1, size_t NumElement2>
void MCSimExecutor::AddInitParameter(std::string so_name, std::string ip_name, const Vector<NumElement1>& mean_or_min,
                                     const Vector<NumElement2>& sigma_or_max, InitParameter::RandomizationType rnd_type) {
  std::string name = so_name + MCSimExecutor::separator_ + ip_name;
  if (ip_list_.find(name) == ip_list_.end()) {
    // Register the parameter in ip_list if it is not registered yet
    auto newparam = new InitParameter();
    newparam->SetRandomConfig(mean_or_min, sigma_or_max, rnd_type);
    ip_list_[name] = newparam;
  } else {
    // Throw error if the parameter is already registered
    throw "More than one definition of one InitParameter.";
  }
}
