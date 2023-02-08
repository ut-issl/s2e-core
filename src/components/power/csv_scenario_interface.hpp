/*
 * @file csv_scenario_interface.hpp
 * @brief Interface to read power related scenario in CSV file
 */

#ifndef S2E_COMPONENTS_POWER_CSV_SCENARIO_INTERFACE_H_
#define S2E_COMPONENTS_POWER_CSV_SCENARIO_INTERFACE_H_

#include <library/math/Vector.hpp>
#include <map>
#include <string>
#include <vector>

typedef std::map<double, double> DoubleBuffer;

/*
 * @class CsvScenarioInterface
 * @brief Interface to read power related scenario in CSV file
 */
class CsvScenarioInterface {
 public:
  /**
   * @fn Initialize
   * @brief Initialize function
   * @param [in] fname: Path to initialize file
   */
  static void Initialize(const std::string fname);

  /**
   * @fn IsCsvScenarioEnabled
   * @brief Return enable flag to use CSV scenario
   */
  static bool IsCsvScenarioEnabled();
  /**
   * @fn GetSunDirectionBody
   * @brief Return sun direction vector in the body fixed frame
   * @param [in] time_query: Time query
   */
  static libra::Vector<3> GetSunDirectionBody(const double time_query);
  /**
   * @fn GetSunFlag
   * @brief Return sun flag
   * @param [in] time_query: Time query
   */
  static bool GetSunFlag(const double time_query);
  /**
   * @fn GetPowerConsumption
   * @brief Return power consumption [W]
   * @param [in] time_query: Time query
   */
  static double GetPowerConsumption(const double time_query);

 private:
  /**
   * @fn ReadCsvData
   * @brief Read CSV data
   * @param [in] filename: Path to CSV file
   * @param [in] ignore_line_num: Number of ignore line
   */
  static std::vector<std::vector<double>> ReadCsvData(const std::string filename, const std::size_t ignore_line_num = 0);
  /**
   * @fn StoreBuffer
   * @brief Store buffer
   * @param [in] buffer_name: Buffer name
   * @param [in] data: Data
   */
  static void StoreBuffer(const std::string buffer_name, const std::vector<std::vector<double>>& data);
  /**
   * @fn GetValueFromBuffer
   * @brief Return value from buffer
   * @param [in] buffer_name: Buffer name
   * @param [in] time_query: Time query
   */
  static double GetValueFromBuffer(const std::string buffer_name, const double time_query);

  static bool is_csv_senario_enabled_;                         //!< Enable flag to use CSV scenario
  static std::map<std::string, unsigned int> buffer_line_id_;  //!< Buffer line ID
  static std::map<std::string, DoubleBuffer> buffers_;         //!< Buffer
};

#endif  // S2E_COMPONENTS_POWER_CSV_SCENARIO_INTERFACE_H_
