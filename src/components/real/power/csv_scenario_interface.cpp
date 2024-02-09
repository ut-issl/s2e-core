/*
 * @file csv_scenario_interface.cpp
 * @brief Interface to read power related scenario in CSV file
 */

#include "csv_scenario_interface.hpp"

#include <initial_setting_file/initialize_file_access.hpp>

bool CsvScenarioInterface::is_csv_scenario_enabled_;
std::map<std::string, unsigned int> CsvScenarioInterface::buffer_line_id_;
std::map<std::string, DoubleBuffer> CsvScenarioInterface::buffers_;

void CsvScenarioInterface::Initialize(const std::string file_name) {
  IniAccess scenario_conf(file_name);
  char Section[30] = "SCENARIO";

  CsvScenarioInterface::is_csv_scenario_enabled_ = scenario_conf.ReadBoolean(Section, "is_csv_scenario_enabled");

  std::string csv_path;
  csv_path = scenario_conf.ReadString(Section, "csv_path");

  buffer_line_id_["sun_dir_b_x"] = 1;
  buffer_line_id_["sun_dir_b_y"] = 2;
  buffer_line_id_["sun_dir_b_z"] = 3;
  buffer_line_id_["sun_flag"] = 4;
  buffer_line_id_["power_consumption"] = 5;

  std::vector<std::vector<double>> data;
  data = ReadCsvData(csv_path, 1);

  for (auto itr = buffer_line_id_.begin(); itr != buffer_line_id_.end(); itr++) {
    StoreBuffer(itr->first, data);
  }
}

bool CsvScenarioInterface::IsCsvScenarioEnabled() { return CsvScenarioInterface::is_csv_scenario_enabled_; }

libra::Vector<3> CsvScenarioInterface::GetSunDirectionBody(const double time_query) {
  libra::Vector<3> sun_dir_b;
  sun_dir_b[0] = GetValueFromBuffer("sun_dir_b_x", time_query);
  sun_dir_b[1] = GetValueFromBuffer("sun_dir_b_y", time_query);
  sun_dir_b[2] = GetValueFromBuffer("sun_dir_b_z", time_query);
  return sun_dir_b;
}

bool CsvScenarioInterface::GetSunFlag(const double time_query) { return (bool)GetValueFromBuffer("sun_flag", time_query); }

double CsvScenarioInterface::GetPowerConsumption(const double time_query) { return GetValueFromBuffer("power_consumption", time_query); }

std::vector<std::vector<double>> CsvScenarioInterface::ReadCsvData(const std::string filename, const std::size_t ignore_line_num) {
  std::ifstream file;
  file.open(filename, std::ios::in);
  if (!file) throw std::invalid_argument(filename + std::string(" cannot be opened."));

  std::string reading_line_buffer;
  for (std::size_t line = 0; line < ignore_line_num; line++) {
    getline(file, reading_line_buffer);
    if (file.eof()) break;
  }

  double num;
  char comma;
  std::vector<std::vector<double>> data;

  while (std::getline(file, reading_line_buffer)) {
    if (reading_line_buffer.size() == 0) break;
    std::vector<double> temp_data;
    std::istringstream is(reading_line_buffer);
    while (is >> num) {
      temp_data.push_back(num);
      is >> comma;
    }
    data.push_back(temp_data);
  }

  return data;
}

void CsvScenarioInterface::StoreBuffer(const std::string buffer_name, const std::vector<std::vector<double>>& data) {
  auto line_num = buffer_line_id_.at(buffer_name);
  for (const auto& line : data) {
    buffers_[buffer_name][line[0]] = line[line_num];
  }
}

double CsvScenarioInterface::GetValueFromBuffer(const std::string buffer_name, const double time_query) {
  double output;
  auto itr = buffers_.at(buffer_name).upper_bound(time_query);
  itr--;
  if (itr == buffers_.at(buffer_name).end()) return 0;
  output = itr->second;
  return output;
}
