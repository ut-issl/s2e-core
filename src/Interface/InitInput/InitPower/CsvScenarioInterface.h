#pragma once

#include "../../../Library/math/Vector.hpp"
#include <map>
#include <string>
#include <vector>

typedef std::map<double, double> DoubleBuffer;

class CsvScenarioInterface {
public:
  static void Initialize(const std::string fname);
  static bool IsCsvScenarioEnabled();
  static libra::Vector<3> GetSunDirectionBody(const double time_query);
  static bool GetSunFlag(const double time_query);
  static double GetPowerConsumption(const double time_query);

private:
  static std::vector<std::vector<double>>
  ReadCsvData(const std::string filename,
              const std::size_t ignore_line_num = 0);
  static void StoreBuffer(const std::string buffer_name,
                          const std::vector<std::vector<double>> &data);
  static double GetValueFromBuffer(const std::string buffer_name,
                                   const double time_query);

  static bool is_csv_senario_enabled_;
  static std::map<std::string, unsigned int> buffer_line_id_;
  static std::map<std::string, DoubleBuffer> buffers_;
};