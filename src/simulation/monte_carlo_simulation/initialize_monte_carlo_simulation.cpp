/**
 * @file initialize_monte_carlo_simulation.cpp
 * @brief Initialize function for Monte-Carlo Simulator
 */

#include "initialize_monte_carlo_simulation.hpp"

#include <cstring>
#include <library/initialize/initialize_file_access.hpp>

#define MAX_CHAR_NUM 256

MonteCarloSimulationExecutor* InitMCSim(std::string file_name) {
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

  enum Phase { FoundNothingYet, FoundSimulationObjectStr, FoundInitParameterStr };
  for (auto so_dot_ip_str : so_dot_ip_str_vec) {
    // Divide the string to SimulationObject and InitMonteCarloParameters
    Phase phase = FoundNothingYet;
    std::stringstream ss(so_dot_ip_str);
    std::string item, so_str, ip_str;
    while (getline(ss, item, MonteCarloSimulationExecutor::separator_)) {
      if (!item.empty()) {
        if (phase == FoundNothingYet) {
          phase = FoundSimulationObjectStr;
          so_str = item;
          so_str_vec.push_back(so_str);
        } else if (phase == FoundSimulationObjectStr) {
          phase = FoundInitParameterStr;
          ip_str = item;
          ip_str_vec.push_back(ip_str);
          break;
        }
      }
    }

    // Read Randomization type
    InitMonteCarloParameters::RandomizationType random_type;
    const static unsigned int buf_size = 256;
    char rnd_type_str[buf_size];
    std::string key_name = so_dot_ip_str + MonteCarloSimulationExecutor::separator_ + "randomization_type";

    ini_file.ReadChar(section, key_name.c_str(), buf_size, rnd_type_str);
    if (!strcmp(rnd_type_str, "NoRandomization"))
      random_type = InitMonteCarloParameters::NoRandomization;
    else if (!strcmp(rnd_type_str, "CartesianUniform"))
      random_type = InitMonteCarloParameters::CartesianUniform;
    else if (!strcmp(rnd_type_str, "CartesianNormal"))
      random_type = InitMonteCarloParameters::CartesianNormal;
    else if (!strcmp(rnd_type_str, "CircularNormalUniform"))
      random_type = InitMonteCarloParameters::CircularNormalUniform;
    else if (!strcmp(rnd_type_str, "CircularNormalNormal"))
      random_type = InitMonteCarloParameters::CircularNormalNormal;
    else if (!strcmp(rnd_type_str, "SphericalNormalUniformUniform"))
      random_type = InitMonteCarloParameters::SphericalNormalUniformUniform;
    else if (!strcmp(rnd_type_str, "SphericalNormalNormal"))
      random_type = InitMonteCarloParameters::SphericalNormalNormal;
    else if (!strcmp(rnd_type_str, "QuaternionUniform"))
      random_type = InitMonteCarloParameters::QuaternionUniform;
    else if (!strcmp(rnd_type_str, "QuaternionNormal"))
      random_type = InitMonteCarloParameters::QuaternionNormal;
    else
      random_type = InitMonteCarloParameters::NoRandomization;

    // Read mean_or_min vector
    key_name = so_dot_ip_str + MonteCarloSimulationExecutor::separator_ + "mean_or_min";
    libra::Vector<3> mean_or_min;
    ini_file.ReadVector(section, key_name.c_str(), mean_or_min);

    // Read sigma_or_max vector
    key_name = so_dot_ip_str + MonteCarloSimulationExecutor::separator_ + "sigma_or_max";
    libra::Vector<3> sigma_or_max;
    ini_file.ReadVector(section, key_name.c_str(), sigma_or_max);

    // Write randomize setting
    monte_carlo_simulator->AddInitMonteCarloParameter(so_str, ip_str, mean_or_min, sigma_or_max, random_type);
  }

  return monte_carlo_simulator;
}
