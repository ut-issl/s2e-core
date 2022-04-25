#include "InitMcSim.hpp"

#include <Interface/InitInput/IniAccess.h>

#include <cstring>

#define MAX_CHAR_NUM 256

MCSimExecutor* InitMCSim(std::string file_name) {
  IniAccess ini_file(file_name);
  const char* section = "MC_EXECUTION";

  unsigned long long total_num_of_executions = ini_file.ReadInt(section, "NumOfExecutions");

  MCSimExecutor* mc_sim = new MCSimExecutor(total_num_of_executions);

  char endis_str[MAX_CHAR_NUM];
  ini_file.ReadChar(section, "MCSimEnabled", MAX_CHAR_NUM, endis_str);
  bool enable = (strcmp(endis_str, "ENABLED") == 0);
  mc_sim->Enable(enable);

  ini_file.ReadChar(section, "LogHistory", MAX_CHAR_NUM, endis_str);
  bool log_history = (strcmp(endis_str, "ENABLED") == 0);
  mc_sim->LogHistory(log_history);

  section = "MC_RANDOMIZATION";
  std::vector<std::string> so_dot_ip_str_vec = ini_file.ReadStrVector(section, "Param");
  std::vector<std::string> so_str_vec, ip_str_vec;

  enum Phase { FoundNothingYet, FoundSimulationObjectStr, FoundInitParameterStr };
  for (auto so_dot_ip_str : so_dot_ip_str_vec) {
    // 文字列をSimulationObjectとInitParameterに分割
    Phase phase = FoundNothingYet;
    std::stringstream ss(so_dot_ip_str);
    std::string item, so_str, ip_str;
    while (getline(ss, item, MCSimExecutor::separator_)) {
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

    // Randomization typeを読みこみ
    InitParameter::RandomizationType rnd_type;
    const static unsigned int buf_size = 256;
    char rnd_type_str[buf_size];
    std::string key_name = so_dot_ip_str + MCSimExecutor::separator_ + "randomization_type";

    ini_file.ReadChar(section, key_name.c_str(), buf_size, rnd_type_str);
    if (!strcmp(rnd_type_str, "NoRandomization"))
      rnd_type = InitParameter::NoRandomization;
    else if (!strcmp(rnd_type_str, "CartesianUniform"))
      rnd_type = InitParameter::CartesianUniform;
    else if (!strcmp(rnd_type_str, "CartesianNormal"))
      rnd_type = InitParameter::CartesianNormal;
    else if (!strcmp(rnd_type_str, "CircularNormalUniform"))
      rnd_type = InitParameter::CircularNormalUniform;
    else if (!strcmp(rnd_type_str, "CircularNormalNormal"))
      rnd_type = InitParameter::CircularNormalNormal;
    else if (!strcmp(rnd_type_str, "SphericalNormalUniformUniform"))
      rnd_type = InitParameter::SphericalNormalUniformUniform;
    else if (!strcmp(rnd_type_str, "SphericalNormalNormal"))
      rnd_type = InitParameter::SphericalNormalNormal;
    else if (!strcmp(rnd_type_str, "QuaternionUniform"))
      rnd_type = InitParameter::QuaternionUniform;
    else if (!strcmp(rnd_type_str, "QuaternionNormal"))
      rnd_type = InitParameter::QuaternionNormal;
    else
      rnd_type = InitParameter::NoRandomization;

    // mean_or_min ベクトルを読み込み
    key_name = so_dot_ip_str + MCSimExecutor::separator_ + "mean_or_min";
    Vector<3> mean_or_min;
    ini_file.ReadVector(section, key_name.c_str(), mean_or_min);

    // sigma_or_max ベクトルを読み込み
    key_name = so_dot_ip_str + MCSimExecutor::separator_ + "sigma_or_max";
    Vector<3> sigma_or_max;
    ini_file.ReadVector(section, key_name.c_str(), sigma_or_max);

    // 乱数設定を書き込み
    mc_sim->AddInitParameter(so_str, ip_str, mean_or_min, sigma_or_max, rnd_type);
  }

  return mc_sim;
}
