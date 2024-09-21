/**
 * @file initialize_monte_carlo_simulation.cpp
 * @brief Initialize function for Monte-Carlo Simulator
 */

#include "initialize_monte_carlo_simulation.hpp"

#include <cstring>
#include <setting_file_reader/initialize_file_access.hpp>

#define MAX_CHAR_NUM 256

MonteCarloSimulationExecutor* InitMonteCarloSimulation(std::string file_name) {
  IniAccess ini_file(file_name);
  const char* section = "MONTE_CARLO_EXECUTION";

  unsigned long long total_num_of_executions = ini_file.ReadInt(section, "number_of_executions");

  MonteCarloSimulationExecutor* monte_carlo_simulator = new MonteCarloSimulationExecutor(total_num_of_executions);

  bool enable = ini_file.ReadEnable(section, "monte_carlo_enable");
  monte_carlo_simulator->SetEnable(enable);

  bool log_history = ini_file.ReadEnable(section, "log_enable");
  monte_carlo_simulator->SetSaveLogHistoryFlag(log_history);

  section = "MONTE_CARLO_RANDOMIZATION";
  std::vector<std::string> so_dot_ip_str_vec = ini_file.ReadStrVector(section, "parameter");
  std::vector<std::string> so_str_vec, ip_str_vec;

  enum Phase { kFoundNothingYet, kFoundSimulationObjectStr, kFoundInitParameterStr };
  for (auto so_dot_ip_str : so_dot_ip_str_vec) {
    // Divide the string to SimulationObject and InitializedMonteCarloParameters
    Phase phase = kFoundNothingYet;
    std::stringstream ss(so_dot_ip_str);
    std::string item, so_str, ip_str;
    while (getline(ss, item, MonteCarloSimulationExecutor::separator_)) {
      if (!item.empty()) {
        if (phase == kFoundNothingYet) {
          phase = kFoundSimulationObjectStr;
          so_str = item;
          so_str_vec.push_back(so_str);
        } else if (phase == kFoundSimulationObjectStr) {
          phase = kFoundInitParameterStr;
          ip_str = item;
          ip_str_vec.push_back(ip_str);
          break;
        }
      }
    }

    // Read Randomization type
    InitializedMonteCarloParameters::RandomizationType random_type;
    const static unsigned int buf_size = 256;
    char rnd_type_str[buf_size];
    std::string key_name = so_dot_ip_str + MonteCarloSimulationExecutor::separator_ + "randomization_type";

    ini_file.ReadChar(section, key_name.c_str(), buf_size, rnd_type_str);
    if (!strcmp(rnd_type_str, "NoRandomization"))
      random_type = InitializedMonteCarloParameters::kNoRandomization;
    else if (!strcmp(rnd_type_str, "CartesianUniform"))
      random_type = InitializedMonteCarloParameters::kCartesianUniform;
    else if (!strcmp(rnd_type_str, "CartesianNormal"))
      random_type = InitializedMonteCarloParameters::kCartesianNormal;
    else if (!strcmp(rnd_type_str, "CircularNormalUniform"))
      random_type = InitializedMonteCarloParameters::kCircularNormalUniform;
    else if (!strcmp(rnd_type_str, "CircularNormalNormal"))
      random_type = InitializedMonteCarloParameters::kCircularNormalNormal;
    else if (!strcmp(rnd_type_str, "SphericalNormalUniformUniform"))
      random_type = InitializedMonteCarloParameters::kSphericalNormalUniformUniform;
    else if (!strcmp(rnd_type_str, "SphericalNormalNormal"))
      random_type = InitializedMonteCarloParameters::kSphericalNormalNormal;
    else if (!strcmp(rnd_type_str, "QuaternionUniform"))
      random_type = InitializedMonteCarloParameters::kQuaternionUniform;
    else if (!strcmp(rnd_type_str, "QuaternionNormal"))
      random_type = InitializedMonteCarloParameters::kQuaternionNormal;
    else
      random_type = InitializedMonteCarloParameters::kNoRandomization;

    // Read mean_or_min vector
    key_name = so_dot_ip_str + MonteCarloSimulationExecutor::separator_ + "mean_or_min";
    math::Vector<3> mean_or_min;
    ini_file.ReadVector(section, key_name.c_str(), mean_or_min);

    // Read sigma_or_max vector
    key_name = so_dot_ip_str + MonteCarloSimulationExecutor::separator_ + "sigma_or_max";
    math::Vector<3> sigma_or_max;
    ini_file.ReadVector(section, key_name.c_str(), sigma_or_max);

    // Write randomize setting
    monte_carlo_simulator->AddInitializedMonteCarloParameter(so_str, ip_str, mean_or_min, sigma_or_max, random_type);
  }

  return monte_carlo_simulator;
}
