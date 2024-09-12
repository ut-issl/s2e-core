/**
 * @file heatload.hpp
 * @brief heatload for thermal node
 */

#ifndef S2E_DYNAMICS_THERMAL_HEATLOAD_HPP_
#define S2E_DYNAMICS_THERMAL_HEATLOAD_HPP_

#include <library/logger/logger.hpp>
#include <string>
#include <vector>

/**
 * @class Heatload
 * @brief Class for calculating heatload value for Node class object at elapsed time
 */
class Heatload {
 public:
  /**
   * @fn Heatload
   * @brief Construct a new Heatload object
   *
   * @param [in] node_id
   * @param [in] time_table_s
   * @param [in] internal_heatload_table_W
   */
  Heatload(const int node_id, const std::vector<double> time_table_s, const std::vector<double> internal_heatload_table_W);
  /**
   * @fn ~Heatload
   * @brief Destroy the Heatload object
   */
  virtual ~Heatload();
  /**
   * @fn CalcInternalHeatload
   * @brief Calculate internal heatload by linear interpolation from defined table
   */
  void CalcInternalHeatload(void);
  /**
   * @fn UpdateTotalHeatload
   * @brief Update total heatload value by summing up all factors
   */
  void UpdateTotalHeatload(void) { total_heatload_W_ = solar_heatload_W_ + albedo_heatload_W_ + internal_heatload_W_ + heater_heatload_W_; }

  // Getter
  /**
   * @fn GetSolarHeatload_W
   * @brief Return Solar Heatload
   */
  inline double GetSolarHeatload_W(void) const { return solar_heatload_W_; }
  /**
   * @fn GetAlbedoHeatload_W
   * @brief Return Albedo Heatload
   */
  inline double GetAlbedoHeatload_W(void) const { return albedo_heatload_W_; }
  /**
   * @fn GetInternalHeatload_W
   * @brief Return Internal Heatload
   */
  inline double GetInternalHeatload_W(void) const { return internal_heatload_W_; }
  /**
   * @fn GetHeaterHeatload_W
   * @brief Return Heater Heatload
   */
  inline double GetHeaterHeatload_W(void) const { return heater_heatload_W_; }
  /**
   * @fn GetTotalHeatload_W
   * @brief Return Total Heatload
   */
  inline double GetTotalHeatload_W(void) const { return total_heatload_W_; }

  // Setter
  /**
   * @brief Set the Time object
   * @param[in] elapsed_time_s
   */
  void SetElapsedTime_s(double elapsed_time_s);
  /**
   * @brief Set Internal Heatload [W]
   * @param[in] internal_heatload_W
   */
  inline void SetInternalHeatload_W(double internal_heatload_W) { internal_heatload_W_ = internal_heatload_W; }
  /**
   * @brief Set Solar Heatload [W]
   * @param[in] solar_heatload_W
   */
  inline void SetSolarHeatload_W(double solar_heatload_W) { solar_heatload_W_ = solar_heatload_W; }
  /**
   * @brief Set Albedo Heatload [W]
   * @param[in] albedo_heatload_W
   */
  inline void SetAlbedoHeatload_W(double albedo_heatload_W) { albedo_heatload_W_ = albedo_heatload_W; }
  /**
   * @brief Set Heater Heatload [W]
   * @param[in] heater_heatload_W
   */
  inline void SetHeaterHeatload_W(double heater_heatload_W) { heater_heatload_W_ = heater_heatload_W; }

 protected:
  double elapsed_time_s_;                          // Elapsed time [s]
  unsigned int node_id_;                           // Node ID to apply heatload
  std::vector<double> time_table_s_;               // Times that internal heatload values are defined [s]
  std::vector<double> internal_heatload_table_W_;  // Defined internal heatload values [W]

  unsigned int elapsed_time_idx_;  // index of time_table_s_ that is closest to elapsed_time_s_
  double solar_heatload_W_;        // Heatload from solar flux [W]
  double albedo_heatload_W_;       // Heatload from albedo flux [W]
  double internal_heatload_W_;     // Heatload from internal dissipation [W]
  double heater_heatload_W_;       // Heatload from heater [W]
  double total_heatload_W_;        // Total heatload [W]

  double time_table_period_s_;      // Value of last element of time_table_s_, which represents the period of the heatload table [s]
  double residual_elapsed_time_s_;  // Residual of dividing elapsed_time_s_ by time_table_period_s_ [s]

  /**
   * @fn AssertHeatloadParams
   * @brief Check if Heatload Parameters are Correct
   */
  void AssertHeatloadParams();
};

/**
 * @fn InitHeatload
 * @brief Initialize Heatload object from csv file
 * @param[in] time_str: str representing time table, read from csv file
 * @param[in] internal_heatload_str: str representing internal heatload table, read from csv file
 * @return Heatload
 */
Heatload InitHeatload(const std::vector<std::string>& time_str, const std::vector<std::string>& internal_heatload_str);

#endif  // S2E_DYNAMICS_THERMAL_HEATLOAD_HPP_
