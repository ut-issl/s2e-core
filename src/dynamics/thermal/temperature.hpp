/**
 * @file temperature.hpp
 * @brief Initialize temperature
 */

#ifndef S2E_DYNAMICS_THERMAL_TEMPERATURE_HPP_
#define S2E_DYNAMICS_THERMAL_TEMPERATURE_HPP_

#include <environment/local/solar_radiation_pressure_environment.hpp>
#include <logger/loggable.hpp>
#include <string>
#include <vector>

#include "heater.hpp"
#include "heater_controller.hpp"
#include "heatload.hpp"
#include "node.hpp"

/**
 * @enum SolarCalcSetting
 * @brief Whether to calculate solar radiation inside simulation
 */
enum class SolarCalcSetting {
  kEnable,
  kDisable,
};

/**
 * @class Temperature
 * @brief class to calculate temperature of all nodes
 */
class Temperature : public ILoggable {
 protected:
  std::vector<std::vector<double>> conductance_matrix_W_K_;  // Coupling of node i and node j by heat conduction [W/K]
  std::vector<std::vector<double>> radiation_matrix_m2_;     // Coupling of node i and node j by thermal radiation [m2]
  std::vector<Node> nodes_;                                  // vector of nodes
  std::vector<Heatload> heatloads_;                          // vector of heatloads
  std::vector<Heater> heaters_;                              // vector of heaters
  std::vector<HeaterController> heater_controllers_;         // vector of heater controllers
  size_t node_num_;                                          // number of nodes
  double propagation_step_s_;                                // propagation step [s]
  double propagation_time_s_;  // Incremented time inside class Temperature [s], finish propagation when reaching end_time
  const SolarRadiationPressureEnvironment* srp_environment_;  // SolarRadiationPressureEnvironment for calculating solar flux
  bool is_calc_enabled_;                                      // Whether temperature calculation is enabled
  SolarCalcSetting solar_calc_setting_;                       // setting for solar calculation
  bool debug_;                                                // Activate debug output or not

  /**
   * @fn CalcRungeOneStep
   * @brief Calculate one step of RK4 for thermal equilibrium equation and update temperatures of nodes
   *
   * @param[in] time_now_s: Current elapsed time [s]
   * @param[in] time_step_s: Time step of RK4 [s]
   * @param[in] sun_direction_b: Sun position in body frame [m]
   * @param[in] node_num: Number of nodes
   */
  void CalcRungeOneStep(double time_now_s, double time_step_s, math::Vector<3> sun_direction_b, size_t node_num);
  /**
   * @fn CalcTemperatureDifferentials
   * @brief Calculate differential of thermal equilibrium equation
   *
   * @param temperatures_K: [UNUSED] Temperatures of each node [K]
   * @param time_now_s: Current elapsed time [s]
   * @param[in] sun_direction_b: Sun direction in body frame
   * @param node_num: Number of nodes
   * @return std::vector<double>: Differential of thermal equilibrium equation at time now
   */
  std::vector<double> CalcTemperatureDifferentials(std::vector<double> temperatures_K, double time_now_s, const math::Vector<3> sun_direction_b,
                                                   size_t node_num);

 public:
  /**
   * @fn Temperature
   * @brief Construct a new Temperature object
   *
   * @param conductance_matrix_W_K: (node_num x node_num) matrix with heat conductance values [W/K]
   * @param radiation_matrix_m2: (node_num x node_num) matrix with radiative connection values [m2]
   * @param nodes: Vector of all nodes included in calculation
   * @param heatloads: Vector of all heatloads included in calculation
   * @param heaters: Vector of all heaters included in calculation
   * @param heater_controllers: Vector of all heater controllers included in calculation
   * @param node_num: Number of nodes
   * @param propagation_step_s: Propagation time step [s]
   * @param is_calc_enabled: Whether calculation is enabled
   * @param solar_calc_setting: Solar calculation settings
   * @param debug: Whether debug is enabled
   */
  Temperature(const std::vector<std::vector<double>> conductance_matrix_W_K, const std::vector<std::vector<double>> radiation_matrix_m2,
              std::vector<Node> nodes, std::vector<Heatload> heatloads, std::vector<Heater> heaters, std::vector<HeaterController> heater_controllers,
              const size_t node_num, const double propagation_step_s, const SolarRadiationPressureEnvironment* srp_environment,
              const bool is_calc_enabled, const SolarCalcSetting solar_calc_setting, const bool debug);
  /**
   * @fn Temperature
   * @brief Construct a new Temperature object, used when thermal calculation is disabled.
   * TODO: Consider whether this overload is required
   */
  Temperature();
  /**
   * @fn ~Temperature
   * @brief Destroy the Temperature object
   *
   */
  virtual ~Temperature();
  /**
   * @fn Propagate
   * @brief Propagate thermal calculation until time_end_s
   *
   * @param[in] sun_position_b_m: Sun position in body frame [m]
   * @param time_end_s: Time to finish propagation [s]
   */
  void Propagate(math::Vector<3> sun_position_b_m, const double time_end_s);

  // Getter
  /**
   * @fn GetNodes
   * @brief Return Nodes
   * @return std::vector<Node>
   */
  inline std::vector<Node> GetNodes() { return nodes_; };
  /**
   * @fn GetHeaterPower_W
   * @brief Return Heater Power of Selected Node
   * @param node_id: Node number to return heater power
   * @return double: Heater power [W]
   */
  double GetHeaterPower_W(size_t node_id);
  /**
   * @fn GetLogHeader
   * @brief Return Log Header
   * @return std::string
   */
  std::string GetLogHeader() const;
  /**
   * @fn GetLogValue
   * @brief Return Log Value
   * @return std::string
   */
  std::string GetLogValue() const;

  /**
   * @fn UpdateHeaterStatus
   * @brief Update all heater status based on heater controller and temperature
   */
  void UpdateHeaterStatus(void);

  /**
   * @fn PrintParams
   * @brief Print parameters of temperature in debug console
   */
  void PrintParams(void);
};

/**
 * @fn InitTemperature
 * @brief Initialize Temperature object from csv file
 * @param[in] file_name: Directory of thermal input files
 * @param[in] rk_prop_step_s: time step interval for temperature propagation integration
 * @return Temperature*
 */
Temperature* InitTemperature(const std::string file_name, const double rk_prop_step_s, const SolarRadiationPressureEnvironment* srp_environment);

#endif  // S2E_DYNAMICS_THERMAL_TEMPERATURE_HPP_
