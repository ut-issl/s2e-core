/**
 * @file skydel_hil.hpp
 * @brief Interface between S2E and the Skydel HIL remote API
 */

#ifndef S2E_SIMULATION_HILS_SKYDEL_HIL_HPP_
#define S2E_SIMULATION_HILS_SKYDEL_HIL_HPP_

#include <cstddef>
#include <cstdint>
#include <math_physics/math/matrix.hpp>
#include <math_physics/math/vector.hpp>
#include <memory>
#include <string>
#include <vector>

namespace Sdx {
class RemoteSimulator;
}  // namespace Sdx

namespace s2e::environment {
class SimulationTime;
}

namespace s2e::geodesy {
class GeodeticPosition;
}

namespace s2e::spacecraft {
class Spacecraft;
}

namespace s2e::simulation {

/**
 * @class SkydelHil
 * @brief Stream S2E spacecraft states to Skydel through the HIL remote API
 */
class SkydelHil {
 public:
  SkydelHil() = default;
  ~SkydelHil();

  SkydelHil(const SkydelHil&) = delete;
  SkydelHil& operator=(const SkydelHil&) = delete;

  /**
   * @brief Read the Skydel HIL configuration
   * @param [in] base_ini_path S2E base initialization file
   * @param [in] number_of_spacecraft Number of S2E spacecraft
   */
  void Initialize(const std::string& base_ini_path, unsigned int number_of_spacecraft);

  /**
   * @brief Connect to Skydel, synchronize the instances, and send the initial states
   */
  void StreamInitialSamples(const std::vector<const spacecraft::Spacecraft*>& spacecraft_list,
                            const environment::SimulationTime& simulation_time);

  /**
   * @brief Send the spacecraft states for the current simulation step
   */
  void StreamStepSamples(const std::vector<const spacecraft::Spacecraft*>& spacecraft_list,
                         const environment::SimulationTime& simulation_time);

  /** @brief Stop and disconnect all Skydel instances */
  void Close();

  /** @brief Return whether Skydel HIL is enabled */
  bool IsEnabled() const { return is_enabled_; }

 private:
  bool is_enabled_ = false;
  std::string skydel_host_;
  int output_period_ms_ = 10;
  int sync_duration_ms_ = 2000;
  int hil_tjoin_ms_ = 200;
  int engine_latency_ms_ = 40;
  int sync_port_ = 4567;
  double simulation_start_timestamp_ms_ = 0.0;
  int64_t last_streamed_elapsed_ms_ = -1;
  int64_t next_warning_elapsed_ms_ = 1000;

  std::vector<unsigned int> spacecraft_ids_;
  std::vector<unsigned int> instance_ids_;
  std::vector<std::string> config_paths_;
  std::vector<std::shared_ptr<Sdx::RemoteSimulator>> simulators_;

  void LoadConfiguration(const std::string& base_ini_path, unsigned int number_of_spacecraft);
  void ValidateVehicleConfigs(unsigned int number_of_spacecraft) const;
  void SetupSimulators(const environment::SimulationTime& simulation_time);
  void StreamSamples(const std::vector<const spacecraft::Spacecraft*>& spacecraft_list,
                     const environment::SimulationTime& simulation_time, bool force, bool wait_for_timestamp);
  void PushSample(size_t index, const spacecraft::Spacecraft& spacecraft, int64_t elapsed_time_ms);

  math::Matrix<3, 3> CalcDcmEcefToNed(const geodesy::GeodeticPosition& geodetic_position) const;
  math::Vector<3> CalcTotalAccelerationEcef_m_s2(const spacecraft::Spacecraft& spacecraft) const;
  void DisplayHilExtrapolationWarnings(Sdx::RemoteSimulator& simulator) const;
};

}  // namespace s2e::simulation

#endif  // S2E_SIMULATION_HILS_SKYDEL_HIL_HPP_
