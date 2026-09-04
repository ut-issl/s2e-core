/**
 * @file skydel_hil.hpp
 * @brief Interface between S2E and the Skydel HIL remote API
 */

#ifndef S2E_SIMULATION_HILS_SKYDEL_HIL_HPP_
#define S2E_SIMULATION_HILS_SKYDEL_HIL_HPP_

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <math_physics/math/vector.hpp>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

namespace Sdx {
class RemoteSimulator;
struct DateTime;
}  // namespace Sdx

namespace s2e::environment {
class SimulationTime;
}

namespace s2e::spacecraft {
class Spacecraft;
}

namespace s2e::simulation {

/**
 * @class SkydelHil
 * @brief Stream S2E spacecraft dynamics to Skydel through the HIL remote API
 *
 * The Skydel API is deliberately hidden behind this interface.  This keeps
 * applications that do not use Skydel independent of the remote API headers
 * and library.
 */
class SkydelHil {
 public:
  SkydelHil();
  ~SkydelHil();

  SkydelHil(const SkydelHil&) = delete;
  SkydelHil& operator=(const SkydelHil&) = delete;

  /**
   * @fn Initialize
   * @brief Read the Skydel HIL configuration
   * @param [in] base_ini_path: S2E base initialization file
   * @param [in] number_of_spacecraft: Number of S2E spacecraft
   */
  void Initialize(const std::string& base_ini_path, unsigned int number_of_spacecraft);

  /**
   * @fn StreamInitialSamples
   * @brief Connect to Skydel and stream the initial spacecraft states
   * @param [in] spacecraft_list: S2E spacecraft list
   * @param [in] simulation_time: S2E simulation time
   */
  void StreamInitialSamples(const std::vector<const spacecraft::Spacecraft*>& spacecraft_list, const environment::SimulationTime& simulation_time);

  /**
   * @fn StreamStepSamples
   * @brief Stream the spacecraft states for the current simulation step
   * @param [in] spacecraft_list: S2E spacecraft list
   * @param [in] simulation_time: S2E simulation time
   */
  void StreamStepSamples(const std::vector<const spacecraft::Spacecraft*>& spacecraft_list, const environment::SimulationTime& simulation_time);

  /**
   * @fn Close
   * @brief Stop streaming and close all Skydel connections
   */
  void Close();

  /**
   * @fn IsEnabled
   * @brief Return whether Skydel HIL is enabled in the initialization file
   */
  bool IsEnabled() const;

 private:
  using RemoteSimulatorPtr = std::unique_ptr<Sdx::RemoteSimulator, void (*)(Sdx::RemoteSimulator*)>;
  using Sample = std::tuple<int64_t, s2e::math::Vector<3>, s2e::math::Vector<3>, s2e::math::Vector<3>, s2e::math::Vector<3>, s2e::math::Vector<3>>;

  static void DeleteRemoteSimulator(Sdx::RemoteSimulator* simulator) noexcept;

  bool is_enabled_ = false;
  std::string skydel_host_;
  int output_period_ms_ = 10;
  int sync_duration_ms_ = 2000;
  int hil_tjoin_ms_ = 200;
  int last_streamed_elapsed_ms_ = -1;

  std::vector<unsigned int> spacecraft_ids_;
  std::vector<unsigned int> instance_ids_;
  std::vector<std::string> vehicle_names_;
  std::vector<std::string> config_paths_;
  std::vector<RemoteSimulatorPtr> simulators_;
  std::thread stream_thread_;

  std::vector<std::deque<Sample>> pending_samples_;
  std::vector<double> simulation_start_timestamp_ms_;
  std::vector<size_t> sent_samples_;
  std::vector<int64_t> last_sent_elapsed_time_ms_;
  bool stop_requested_ = false;
  bool stream_failed_ = false;
  std::mutex queue_mutex_;
  std::condition_variable queue_condition_variable_;

  mutable std::mutex mutex_;
  mutable std::mutex error_mutex_;
  std::string worker_error_message_;

  void LoadConfiguration(const std::string& base_ini_path, unsigned int number_of_spacecraft);
  void RunStream(std::vector<Sample> initial_samples, const Sdx::DateTime& start_time, int duration_sec);
  void StreamSamples(const std::vector<const spacecraft::Spacecraft*>& spacecraft_list, const environment::SimulationTime& simulation_time,
                     bool force);
  Sample BuildSample(const spacecraft::Spacecraft& spacecraft, int64_t elapsed_time_ms) const;
  void PushSample(size_t index, const Sample& sample);
  void EnqueueSample(size_t index, const Sample& sample);
  void WaitUntilSamplesSent(int64_t elapsed_time_ms);
  void CleanupSimulator(size_t index) noexcept;
  void CloseUnlocked();
  void RequestStop();
  void SetWorkerError(const std::string& message);
  void ThrowIfWorkerError() const;
  void ValidateVehicleConfigs(unsigned int number_of_spacecraft) const;
};

}  // namespace s2e::simulation

#endif  // S2E_SIMULATION_HILS_SKYDEL_HIL_HPP_
