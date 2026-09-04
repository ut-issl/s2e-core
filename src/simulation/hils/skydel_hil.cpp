/**
 * @file skydel_hil.cpp
 * @brief Interface between S2E and the Skydel HIL remote API
 */

#include "skydel_hil.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <environment/global/physical_constants.hpp>
#include <environment/global/simulation_time.hpp>
#include <iostream>
#include <math_physics/math/matrix_vector.hpp>
#include <math_physics/math/quaternion.hpp>
#include <math_physics/time_system/date_time_format.hpp>
#include <math_physics/time_system/epoch_time.hpp>
#include <math_physics/time_system/gps_time.hpp>
#include <memory>
#include <mutex>
#include <set>
#include <setting_file_reader/initialize_file_access.hpp>
#include <simulation/spacecraft/spacecraft.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

namespace {

bool IsIniNullString(const std::string& value) { return value.empty() || value == "NULL"; }

std::string ReadHilIniPath(const std::string& base_ini_path) {
  s2e::setting_file_reader::IniAccess base_ini(base_ini_path);
  return base_ini.ReadString("SIMULATION_SETTINGS", "skydel_hil_file");
}

}  // namespace

#ifdef USE_SKYDEL

#include <all_commands.h>
#include <attitude.h>
#include <date_time.h>
#include <ecef.h>
#include <hil_helper.h>
#include <remote_simulator.h>

namespace {

constexpr bool kVerbose = false;
constexpr int kSkydelLatencyMs = 40;
constexpr int kSkydelStreamingBufferSizeMs = 200;
constexpr uint64_t kLeapSecondsUtcToGps = 18;
constexpr double kGpsWeekSeconds = 604800.0;

std::string ReadVehicleSectionName(const size_t index) {
  std::ostringstream oss;
  oss << "VEHICLE_" << index;
  return oss.str();
}

s2e::math::Vector<3> CalcTotalAcceleration_ecef_m_s2(const s2e::spacecraft::Spacecraft& spacecraft) {
  const auto& dynamics = spacecraft.GetDynamics();
  const auto& orbit = dynamics.GetOrbit();
  const auto& celestial_information = spacecraft.GetLocalEnvironment().GetCelestialInformation().GetGlobalInformation();

  s2e::math::Vector<3> acceleration_i_m_s2(0.0);
  if (orbit.GetPropagateMode() != s2e::dynamics::orbit::OrbitPropagateMode::kRelativeOrbit) {
    auto& disturbances = const_cast<s2e::disturbances::Disturbances&>(spacecraft.GetDisturbances());
    auto& components = const_cast<s2e::spacecraft::InstalledComponents&>(spacecraft.GetInstalledComponents());

    acceleration_i_m_s2 += disturbances.GetAcceleration_i_m_s2();

    s2e::math::Vector<3> force_b_N = disturbances.GetForce_b_N();
    force_b_N += components.GenerateForce_b_N();

    const double spacecraft_mass_kg = spacecraft.GetStructure().GetKinematicsParameters().GetMass_kg();
    if (spacecraft_mass_kg > 0.0) {
      s2e::math::Vector<3> force_i_N = dynamics.GetAttitude().GetQuaternion_i2b().InverseFrameConversion(force_b_N);
      force_i_N /= spacecraft_mass_kg;
      acceleration_i_m_s2 += force_i_N;
    }
  }

  const s2e::math::Vector<3> position_i_m = orbit.GetPosition_i_m();
  const double radius_m = position_i_m.CalcNorm();
  if (radius_m > 0.0) {
    const double gravity_constant_m3_s2 = celestial_information.GetCenterBodyGravityConstant_m3_s2();
    acceleration_i_m_s2 += -(gravity_constant_m3_s2 / (radius_m * radius_m * radius_m)) * position_i_m;
  }

  const auto dcm_i_to_ecef = celestial_information.GetEarthRotation().GetDcmJ2000ToEcef();
  const s2e::math::Vector<3> position_ecef_m = orbit.GetPosition_ecef_m();
  const s2e::math::Vector<3> velocity_ecef_m_s = orbit.GetVelocity_ecef_m_s();
  s2e::math::Vector<3> earth_angular_velocity_ecef_rad_s(0.0);
  earth_angular_velocity_ecef_rad_s[2] = s2e::environment::earth_mean_angular_velocity_rad_s;

  s2e::math::Vector<3> acceleration_ecef_m_s2 = dcm_i_to_ecef * acceleration_i_m_s2;
  acceleration_ecef_m_s2 -= 2.0 * s2e::math::OuterProduct(earth_angular_velocity_ecef_rad_s, velocity_ecef_m_s);
  acceleration_ecef_m_s2 -=
      s2e::math::OuterProduct(earth_angular_velocity_ecef_rad_s, s2e::math::OuterProduct(earth_angular_velocity_ecef_rad_s, position_ecef_m));

  return acceleration_ecef_m_s2;
}

s2e::math::Matrix<3, 3> CalcDcmEcefToNed(const s2e::geodesy::GeodeticPosition& geodetic_position) {
  // S2E's local topographic frame is ENU.  Skydel's pushEcefNed API uses NED.
  const auto dcm_ecef_to_enu = geodetic_position.GetQuaternionXcxfToLtc().ConvertToDcm();
  s2e::math::Matrix<3, 3> dcm_ecef_to_ned;
  for (size_t column = 0; column < 3; ++column) {
    dcm_ecef_to_ned[0][column] = dcm_ecef_to_enu[1][column];
    dcm_ecef_to_ned[1][column] = dcm_ecef_to_enu[0][column];
    dcm_ecef_to_ned[2][column] = -dcm_ecef_to_enu[2][column];
  }
  return dcm_ecef_to_ned;
}

void displayHilExtrapolationWarnings(Sdx::RemoteSimulator& sim) {
  const bool is_verbose = sim.isVerbose();
  sim.setVerbose(false);
  const auto result = Sdx::Cmd::GetHilExtrapolationStateResult::dynamicCast(sim.call(Sdx::Cmd::GetHilExtrapolationState::create()));
  if (result->state() == Sdx::HilExtrapolationState::NonDeterministic) {
    std::cout << "Warning: HIL non deterministic extrapolation at millisecond " << result->elapsedTime() << std::endl;
  } else if (result->state() == Sdx::HilExtrapolationState::Snap) {
    std::cout << "Warning: HIL position snap at millisecond " << result->elapsedTime() << std::endl;
  }
  sim.setVerbose(is_verbose);
}

}  // namespace

#endif

namespace s2e::simulation {

SkydelHil::SkydelHil() = default;

SkydelHil::~SkydelHil() { Close(); }

void SkydelHil::DeleteRemoteSimulator(Sdx::RemoteSimulator* simulator) noexcept {
#ifdef USE_SKYDEL
  delete simulator;
#else
  (void)simulator;
#endif
}

void SkydelHil::Initialize(const std::string& base_ini_path, const unsigned int number_of_spacecraft) {
  std::lock_guard<std::mutex> lock(mutex_);
  CloseUnlocked();
  {
    std::lock_guard<std::mutex> error_lock(error_mutex_);
    worker_error_message_.clear();
  }
  LoadConfiguration(base_ini_path, number_of_spacecraft);
}

void SkydelHil::StreamInitialSamples(const std::vector<const s2e::spacecraft::Spacecraft*>& spacecraft_list,
                                     const s2e::environment::SimulationTime& simulation_time) {
  std::lock_guard<std::mutex> lock(mutex_);
  ThrowIfWorkerError();
  if (!is_enabled_) {
    return;
  }

#ifdef USE_SKYDEL
  if (!stream_thread_.joinable()) {
    std::vector<Sample> initial_samples;
    initial_samples.reserve(spacecraft_ids_.size());
    for (const unsigned int spacecraft_id : spacecraft_ids_) {
      if (spacecraft_id >= spacecraft_list.size()) {
        throw std::runtime_error("Skydel HIL spacecraft_id is out of range");
      }
      const s2e::spacecraft::Spacecraft* spacecraft = spacecraft_list[spacecraft_id];
      if (spacecraft == nullptr) {
        throw std::runtime_error("Skydel HIL spacecraft pointer is null");
      }
      initial_samples.push_back(BuildSample(*spacecraft, static_cast<int64_t>(std::llround(simulation_time.GetElapsedTime_s() * 1000.0))));
    }

    const s2e::time_system::DateTime utc_start_time(
        static_cast<size_t>(simulation_time.GetStartYear()), static_cast<size_t>(simulation_time.GetStartMonth()),
        static_cast<size_t>(simulation_time.GetStartDay()), static_cast<size_t>(simulation_time.GetStartHour()),
        static_cast<size_t>(simulation_time.GetStartMinute()), simulation_time.GetStartSecond());
    const s2e::time_system::DateTime gps_start_time(s2e::time_system::EpochTime(utc_start_time) +
                                                    s2e::time_system::EpochTime(kLeapSecondsUtcToGps, 0.0));
    const Sdx::DateTime start_time(static_cast<int>(gps_start_time.GetYear()), static_cast<int>(gps_start_time.GetMonth()),
                                   static_cast<int>(gps_start_time.GetDay()), static_cast<int>(gps_start_time.GetHour()),
                                   static_cast<int>(gps_start_time.GetMinute()), static_cast<int>(std::llround(gps_start_time.GetSecond())));
    const int duration_sec = std::max(1, static_cast<int>(std::llround(simulation_time.GetEndTime_s())));
    const int elapsed_ms = static_cast<int>(std::llround(simulation_time.GetElapsedTime_s() * 1000.0));

    std::cout << "[SkydelHIL] time_source=simulation_start_time_utc simulation_start_time_utc=" << utc_start_time.GetAsString()
              << " leap_seconds_utc_to_gps=" << kLeapSecondsUtcToGps << " skydel_gps_start_datetime=" << gps_start_time.GetAsString()
              << " gps_week_seconds=" << kGpsWeekSeconds << std::endl;

    {
      std::lock_guard<std::mutex> queue_lock(queue_mutex_);
      pending_samples_.assign(simulators_.size(), std::deque<Sample>());
      simulation_start_timestamp_ms_.assign(simulators_.size(), 0.0);
      sent_samples_.assign(simulators_.size(), 0);
      last_sent_elapsed_time_ms_.assign(simulators_.size(), -1);
      stop_requested_ = false;
      stream_failed_ = false;
    }

    stream_thread_ = std::thread(&SkydelHil::RunStream, this, std::move(initial_samples), start_time, duration_sec);
    WaitUntilSamplesSent(elapsed_ms);
    last_streamed_elapsed_ms_ = elapsed_ms;
  } else {
    StreamSamples(spacecraft_list, simulation_time, true);
  }
#else
  (void)spacecraft_list;
  (void)simulation_time;
#endif
}

void SkydelHil::StreamStepSamples(const std::vector<const s2e::spacecraft::Spacecraft*>& spacecraft_list,
                                  const s2e::environment::SimulationTime& simulation_time) {
  std::lock_guard<std::mutex> lock(mutex_);
  ThrowIfWorkerError();
  if (!is_enabled_) {
    return;
  }

#ifdef USE_SKYDEL
  StreamSamples(spacecraft_list, simulation_time, false);
#else
  (void)spacecraft_list;
  (void)simulation_time;
#endif
}

void SkydelHil::Close() {
  std::lock_guard<std::mutex> lock(mutex_);
  CloseUnlocked();
}

bool SkydelHil::IsEnabled() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return is_enabled_;
}

void SkydelHil::LoadConfiguration(const std::string& base_ini_path, const unsigned int number_of_spacecraft) {
  const std::string hil_ini_path = ReadHilIniPath(base_ini_path);
  if (IsIniNullString(hil_ini_path)) {
    is_enabled_ = false;
    return;
  }

  s2e::setting_file_reader::IniAccess hil_ini(hil_ini_path);
  is_enabled_ = hil_ini.ReadEnable("SKYDEL_HIL", "enable");
  if (!is_enabled_) {
    std::cout << "[SkydelHIL] disabled: " << hil_ini_path << std::endl;
    return;
  }

#ifndef USE_SKYDEL
  (void)number_of_spacecraft;
  throw std::runtime_error("Skydel HIL is enabled, but this build was configured without USE_SKYDEL");
#else
  skydel_host_ = hil_ini.ReadString("SKYDEL_HIL", "skydel_host");
  output_period_ms_ = hil_ini.ReadInt("SKYDEL_HIL", "output_period_ms");
  sync_duration_ms_ = hil_ini.ReadInt("SKYDEL_HIL", "sync_duration_ms");
  hil_tjoin_ms_ = hil_ini.ReadInt("SKYDEL_HIL", "hil_tjoin_ms");
  const int number_of_vehicles = hil_ini.ReadInt("SKYDEL_HIL", "number_of_vehicles");

  if (output_period_ms_ <= 0 || sync_duration_ms_ <= 0 || hil_tjoin_ms_ <= 0) {
    throw std::runtime_error("Skydel HIL parameters must be positive");
  }
  if (number_of_vehicles <= 0) {
    throw std::runtime_error("Skydel HIL requires at least one vehicle");
  }

  spacecraft_ids_.clear();
  instance_ids_.clear();
  vehicle_names_.clear();
  config_paths_.clear();
  simulators_.clear();

  spacecraft_ids_.reserve(static_cast<size_t>(number_of_vehicles));
  instance_ids_.reserve(static_cast<size_t>(number_of_vehicles));
  vehicle_names_.reserve(static_cast<size_t>(number_of_vehicles));
  config_paths_.reserve(static_cast<size_t>(number_of_vehicles));
  simulators_.reserve(static_cast<size_t>(number_of_vehicles));
  for (int i = 0; i < number_of_vehicles; ++i) {
    const std::string section = ReadVehicleSectionName(static_cast<size_t>(i));
    spacecraft_ids_.push_back(static_cast<unsigned int>(hil_ini.ReadInt(section.c_str(), "spacecraft_id")));
    instance_ids_.push_back(static_cast<unsigned int>(hil_ini.ReadInt(section.c_str(), "instance")));
    vehicle_names_.push_back(hil_ini.ReadString(section.c_str(), "name"));
    config_paths_.push_back(hil_ini.ReadString(section.c_str(), "config_path"));
    simulators_.emplace_back(nullptr, &SkydelHil::DeleteRemoteSimulator);
  }

  ValidateVehicleConfigs(number_of_spacecraft);
  std::cout << "[SkydelHIL] enabled: " << hil_ini_path << std::endl;
#endif
}

#ifdef USE_SKYDEL

void SkydelHil::RunStream(std::vector<Sample> initial_samples, const Sdx::DateTime& start_time, const int duration_sec) {
  try {
    for (size_t index = 0; index < simulators_.size(); ++index) {
      simulators_[index].reset(new Sdx::RemoteSimulator());
      Sdx::RemoteSimulator& sim = *simulators_[index];
      sim.setVerbose(kVerbose);
      if (!sim.connect(skydel_host_, static_cast<int>(instance_ids_[index]))) {
        throw std::runtime_error("failed to connect to Skydel instance");
      }
      sim.setHilStreamingCheckEnabled(true);

      // These checks mirror the official runExampleHilRealtime sample.
      if (Sdx::Cmd::GetEngineLatencyResult::dynamicCast(sim.call(Sdx::Cmd::GetEngineLatency::create()))->latency() != kSkydelLatencyMs) {
        throw std::runtime_error("unexpected Skydel engine latency");
      }
      if (Sdx::Cmd::GetStreamingBufferResult::dynamicCast(sim.call(Sdx::Cmd::GetStreamingBuffer::create()))->size() != kSkydelStreamingBufferSizeMs) {
        throw std::runtime_error("unexpected Skydel streaming buffer size");
      }

      std::cout << "[SkydelHIL] opening instance=" << instance_ids_[index] << " config_path=" << config_paths_[index] << std::endl;
      sim.call(Sdx::Cmd::Open::create(config_paths_[index], true));
      sim.call(Sdx::Cmd::SetVehicleTrajectory::create("HIL"));
      sim.call(Sdx::Cmd::SetHilTjoin::create(hil_tjoin_ms_));
      sim.call(Sdx::Cmd::SetStartTimeMode::create("Custom"));
      sim.call(Sdx::Cmd::SetGpsStartTime::create(start_time));
      sim.call(Sdx::Cmd::SetDuration::create(duration_sec));
      sim.call(Sdx::Cmd::EnableLogRaw::create(false));
      sim.call(Sdx::Cmd::EnableLogHILInput::create(true));
    }

    // Enable PPS synchronization before the official PPS sequence.  The main
    // instance drives PPS and worker instances follow it through Skydel's
    // worker synchronization.
    Sdx::RemoteSimulator& sim = *simulators_.front();
    sim.call(Sdx::Cmd::EnableMainInstanceSync::create(true));
    for (size_t index = 1; index < simulators_.size(); ++index) {
      simulators_[index]->call(Sdx::Cmd::EnableWorkerInstanceSync::create(true));
    }
    sim.call(Sdx::Cmd::ArmPPS::create());
    sim.call(Sdx::Cmd::WaitAndResetPPS::create());
    sim.call(Sdx::Cmd::StartPPS::create(sync_duration_ms_));

    std::vector<double> pps0_timestamps_ms;
    pps0_timestamps_ms.reserve(simulators_.size());
    for (size_t index = 0; index < simulators_.size(); ++index) {
      const auto result = Sdx::Cmd::GetComputerSystemTimeSinceEpochAtPps0Result::dynamicCast(
          simulators_[index]->call(Sdx::Cmd::GetComputerSystemTimeSinceEpochAtPps0::create()));
      if (!result) {
        throw std::runtime_error("failed to obtain PPS0 timestamp from Skydel");
      }
      const double pps0_timestamp_ms = result->milliseconds();
      pps0_timestamps_ms.push_back(pps0_timestamp_ms);
      simulation_start_timestamp_ms_[index] = pps0_timestamp_ms + static_cast<double>(sync_duration_ms_);
    }

    if (!pps0_timestamps_ms.empty()) {
      const auto minmax = std::minmax_element(pps0_timestamps_ms.begin(), pps0_timestamps_ms.end());
      std::cout << "[SkydelHIL][PPS0] max_diff_ms=" << (*minmax.second - *minmax.first) << std::endl;
    }

    // The official sample sends the first position immediately after
    // StartPPS, then schedules every following position by its timestamp.
    for (size_t index = 0; index < initial_samples.size(); ++index) {
      PushSample(index, initial_samples[index]);
    }

    double warningTimeMs = 0.0;
    bool stop = false;
    while (!stop) {
      for (size_t index = 0; index < simulators_.size(); ++index) {
        Sample sample;
        {
          std::unique_lock<std::mutex> lock(queue_mutex_);
          queue_condition_variable_.wait(lock, [&] { return stop_requested_ || !pending_samples_[index].empty(); });
          if (stop_requested_) {
            stop = true;
            break;
          }
          sample = std::move(pending_samples_[index].front());
          pending_samples_[index].pop_front();
        }

        PushSample(index, sample);
        const double elapsedMs = static_cast<double>(std::get<0>(sample));
        if (elapsedMs > warningTimeMs + 1000.0) {
          warningTimeMs = elapsedMs;
          displayHilExtrapolationWarnings(*simulators_[index]);
        }
      }
    }

    for (auto& simulator : simulators_) {
      if (simulator) {
        simulator->stop();
      }
    }
  } catch (const std::exception& exception) {
    std::ostringstream oss;
    oss << "Skydel HIL streaming failed: " << exception.what();
    SetWorkerError(oss.str());
  }

  for (size_t index = 0; index < simulators_.size(); ++index) {
    CleanupSimulator(index);
  }
}

#endif

SkydelHil::Sample SkydelHil::BuildSample(const s2e::spacecraft::Spacecraft& spacecraft, const int64_t elapsed_time_ms) const {
#ifdef USE_SKYDEL
  const auto& dynamics = spacecraft.GetDynamics();
  const auto& orbit = dynamics.GetOrbit();
  const auto& celestial_information = spacecraft.GetLocalEnvironment().GetCelestialInformation().GetGlobalInformation();
  const auto dcm_i_to_ecef = celestial_information.GetEarthRotation().GetDcmJ2000ToEcef();

  const s2e::math::Vector<3> acceleration_ecef_m_s2 = CalcTotalAcceleration_ecef_m_s2(spacecraft);

  const s2e::math::Vector<3> position_ecef_m = orbit.GetPosition_ecef_m();
  const s2e::math::Vector<3> velocity_ecef_m_s = orbit.GetVelocity_ecef_m_s();

  const auto dcm_ecef_to_ned = CalcDcmEcefToNed(orbit.GetGeodeticPosition());
  const auto dcm_ned_to_i = dcm_i_to_ecef.Transpose() * dcm_ecef_to_ned.Transpose();
  const auto dcm_ned_to_b = dynamics.GetAttitude().GetQuaternion_i2b().ConvertToDcm() * dcm_ned_to_i;
  const auto euler_roll_pitch_yaw_rad = s2e::math::Quaternion::ConvertFromDcm(dcm_ned_to_b).Normalize().ConvertToEuler();
  s2e::math::Vector<3> attitude_ypr_rad(0.0);
  attitude_ypr_rad[0] = euler_roll_pitch_yaw_rad[2];
  attitude_ypr_rad[1] = euler_roll_pitch_yaw_rad[1];
  attitude_ypr_rad[2] = euler_roll_pitch_yaw_rad[0];

  const auto angular_velocity_b_rad_s = dynamics.GetAttitude().GetAngularVelocity_b_rad_s();
  s2e::math::Vector<3> angular_velocity_ypr_rad_s(0.0);
  angular_velocity_ypr_rad_s[0] = angular_velocity_b_rad_s[2];
  angular_velocity_ypr_rad_s[1] = angular_velocity_b_rad_s[1];
  angular_velocity_ypr_rad_s[2] = angular_velocity_b_rad_s[0];

  return std::make_tuple(elapsed_time_ms, position_ecef_m, velocity_ecef_m_s, acceleration_ecef_m_s2, attitude_ypr_rad, angular_velocity_ypr_rad_s);
#else
  (void)spacecraft;
  return std::make_tuple(elapsed_time_ms, s2e::math::Vector<3>(0.0), s2e::math::Vector<3>(0.0), s2e::math::Vector<3>(0.0), s2e::math::Vector<3>(0.0),
                         s2e::math::Vector<3>(0.0));
#endif
}

void SkydelHil::PushSample(const size_t index, const Sample& sample) {
#ifdef USE_SKYDEL
  const int64_t elapsed_time_ms = std::get<0>(sample);
  if (sent_samples_[index] > 0) {
    Sdx::preciseSleepUntilMs(simulation_start_timestamp_ms_[index] + static_cast<double>(elapsed_time_ms));
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (stop_requested_) {
      return;
    }
  }

  const auto& position_ecef_m = std::get<1>(sample);
  const auto& velocity_ecef_m_s = std::get<2>(sample);
  const auto& acceleration_ecef_m_s2 = std::get<3>(sample);
  const auto& attitude_ypr_rad = std::get<4>(sample);
  const auto& angular_velocity_ypr_rad_s = std::get<5>(sample);

  const Sdx::Ecef position(position_ecef_m[0], position_ecef_m[1], position_ecef_m[2]);
  const Sdx::Attitude attitude(attitude_ypr_rad[0], attitude_ypr_rad[1], attitude_ypr_rad[2]);
  const Sdx::Ecef velocity(velocity_ecef_m_s[0], velocity_ecef_m_s[1], velocity_ecef_m_s[2]);
  const Sdx::Attitude angular_velocity(angular_velocity_ypr_rad_s[0], angular_velocity_ypr_rad_s[1], angular_velocity_ypr_rad_s[2]);
  const Sdx::Ecef acceleration(acceleration_ecef_m_s2[0], acceleration_ecef_m_s2[1], acceleration_ecef_m_s2[2]);
  const Sdx::Attitude angular_acceleration(0.0, 0.0, 0.0);

  Sdx::RemoteSimulator& sim = *simulators_[index];
  if (!sim.pushEcefNed(static_cast<double>(elapsed_time_ms), position, attitude, velocity, angular_velocity, acceleration, angular_acceleration)) {
    throw std::runtime_error("failed to push HIL sample to Skydel");
  }

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    ++sent_samples_[index];
    last_sent_elapsed_time_ms_[index] = elapsed_time_ms;
  }
  queue_condition_variable_.notify_all();
#else
  (void)index;
  (void)sample;
#endif
}

void SkydelHil::EnqueueSample(const size_t index, const Sample& sample) {
#ifdef USE_SKYDEL
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    pending_samples_[index].push_back(sample);
  }
  queue_condition_variable_.notify_all();
#else
  (void)index;
  (void)sample;
#endif
}

void SkydelHil::StreamSamples(const std::vector<const s2e::spacecraft::Spacecraft*>& spacecraft_list,
                              const s2e::environment::SimulationTime& simulation_time, const bool force) {
#ifdef USE_SKYDEL
  const int elapsed_ms = static_cast<int>(std::llround(simulation_time.GetElapsedTime_s() * 1000.0));
  if (!force) {
    if (last_streamed_elapsed_ms_ >= 0 && elapsed_ms <= last_streamed_elapsed_ms_) {
      return;
    }
    if (last_streamed_elapsed_ms_ >= 0 && elapsed_ms - last_streamed_elapsed_ms_ < output_period_ms_) {
      return;
    }
  }

  for (size_t index = 0; index < spacecraft_ids_.size(); ++index) {
    if (spacecraft_ids_[index] >= spacecraft_list.size()) {
      throw std::runtime_error("Skydel HIL spacecraft_id is out of range");
    }
    const s2e::spacecraft::Spacecraft* spacecraft = spacecraft_list[spacecraft_ids_[index]];
    if (spacecraft == nullptr) {
      throw std::runtime_error("Skydel HIL spacecraft pointer is null");
    }
    EnqueueSample(index, BuildSample(*spacecraft, elapsed_ms));
  }

  WaitUntilSamplesSent(elapsed_ms);
  last_streamed_elapsed_ms_ = elapsed_ms;
#else
  (void)spacecraft_list;
  (void)simulation_time;
  (void)force;
#endif
}

void SkydelHil::WaitUntilSamplesSent(const int64_t elapsed_time_ms) {
#ifdef USE_SKYDEL
  std::unique_lock<std::mutex> lock(queue_mutex_);
  queue_condition_variable_.wait(lock, [&] {
    if (stop_requested_ || stream_failed_) {
      return true;
    }
    for (const int64_t last_sent_elapsed_time_ms : last_sent_elapsed_time_ms_) {
      if (last_sent_elapsed_time_ms < elapsed_time_ms) {
        return false;
      }
    }
    return true;
  });
  lock.unlock();
  ThrowIfWorkerError();
#else
  (void)elapsed_time_ms;
#endif
}

void SkydelHil::CleanupSimulator(const size_t index) noexcept {
#ifdef USE_SKYDEL
  if (index >= simulators_.size() || !simulators_[index]) {
    return;
  }
  try {
    simulators_[index]->stop();
  } catch (...) {
  }
  try {
    simulators_[index]->disconnect();
  } catch (...) {
  }
#else
  (void)index;
#endif
  if (index < simulators_.size()) {
    simulators_[index].reset();
  }
}

void SkydelHil::CloseUnlocked() {
  RequestStop();
  if (stream_thread_.joinable()) {
    stream_thread_.join();
  }

  for (size_t index = 0; index < simulators_.size(); ++index) {
    CleanupSimulator(index);
  }

  spacecraft_ids_.clear();
  instance_ids_.clear();
  vehicle_names_.clear();
  config_paths_.clear();
  simulators_.clear();
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    pending_samples_.clear();
    simulation_start_timestamp_ms_.clear();
    sent_samples_.clear();
    last_sent_elapsed_time_ms_.clear();
    stop_requested_ = false;
    stream_failed_ = false;
  }
  last_streamed_elapsed_ms_ = -1;
  is_enabled_ = false;
}

void SkydelHil::RequestStop() {
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    stop_requested_ = true;
  }
  queue_condition_variable_.notify_all();
}

void SkydelHil::SetWorkerError(const std::string& message) {
  {
    std::lock_guard<std::mutex> lock(error_mutex_);
    if (worker_error_message_.empty()) {
      worker_error_message_ = message;
    }
  }
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    stream_failed_ = true;
    stop_requested_ = true;
  }
  queue_condition_variable_.notify_all();
}

void SkydelHil::ThrowIfWorkerError() const {
  std::lock_guard<std::mutex> lock(error_mutex_);
  if (!worker_error_message_.empty()) {
    throw std::runtime_error(worker_error_message_);
  }
}

void SkydelHil::ValidateVehicleConfigs(const unsigned int number_of_spacecraft) const {
#ifdef USE_SKYDEL
  if (IsIniNullString(skydel_host_)) {
    throw std::runtime_error("Skydel HIL host must not be empty");
  }

  std::set<unsigned int> spacecraft_ids;
  std::set<unsigned int> instances;
  for (size_t index = 0; index < spacecraft_ids_.size(); ++index) {
    if (IsIniNullString(vehicle_names_[index])) {
      throw std::runtime_error("Skydel HIL vehicle name must not be empty");
    }
    if (IsIniNullString(config_paths_[index])) {
      std::ostringstream oss;
      oss << "Skydel HIL config_path must be set for spacecraft_id=" << spacecraft_ids_[index] << " instance=" << instance_ids_[index];
      throw std::runtime_error(oss.str());
    }
    if (spacecraft_ids_[index] >= number_of_spacecraft) {
      throw std::runtime_error("Skydel HIL spacecraft_id exceeds the number of simulated spacecraft");
    }
    if (!spacecraft_ids.insert(spacecraft_ids_[index]).second) {
      throw std::runtime_error("Skydel HIL spacecraft_id must be unique");
    }
    if (!instances.insert(instance_ids_[index]).second) {
      throw std::runtime_error("Skydel HIL instance must be unique");
    }
  }
#else
  (void)number_of_spacecraft;
#endif
}

}  // namespace s2e::simulation
