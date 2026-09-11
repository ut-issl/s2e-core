/**
 * @file skydel_hil.cpp
 * @brief Interface between S2E and the Skydel HIL remote API
 */

#include "skydel_hil.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <environment/global/simulation_time.hpp>
#include <iostream>
#include <math_physics/math/matrix_vector.hpp>
#include <math_physics/math/quaternion.hpp>
#include <math_physics/time_system/date_time_format.hpp>
#include <math_physics/time_system/epoch_time.hpp>
#include <math_physics/time_system/gps_time.hpp>
#include <set>
#include <setting_file_reader/initialize_file_access.hpp>
#include <simulation/spacecraft/spacecraft.hpp>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <all_commands.h>
#include <attitude.h>
#include <date_time.h>
#include <ecef.h>
#include <hil_helper.h>
#include <remote_simulator.h>

namespace s2e::simulation {

SkydelHil::~SkydelHil() { Close(); }

void SkydelHil::Initialize(const std::string& base_ini_path, const unsigned int number_of_spacecraft) {
  Close();
  LoadConfiguration(base_ini_path, number_of_spacecraft);
}

void SkydelHil::StreamInitialSamples(const std::vector<const spacecraft::Spacecraft*>& spacecraft_list,
                                     const environment::SimulationTime& simulation_time) {
  if (!is_enabled_) return;

  try {
    for (const unsigned int spacecraft_id : spacecraft_ids_) {
      if (spacecraft_id >= spacecraft_list.size() || spacecraft_list[spacecraft_id] == nullptr) {
        throw std::runtime_error("Invalid spacecraft_id in Skydel HIL configuration");
      }
      if (spacecraft_list[spacecraft_id]->GetDynamics().GetOrbit().GetPropagateMode() !=
          s2e::dynamics::orbit::OrbitPropagateMode::kRk4) {
        throw std::runtime_error("Skydel HIL supports only RK4 orbit propagation");
      }
    }

    SetupSimulators(simulation_time);
    StreamSamples(spacecraft_list, simulation_time, true, false);
  } catch (...) {
    Close();
    throw;
  }
}

void SkydelHil::StreamStepSamples(const std::vector<const spacecraft::Spacecraft*>& spacecraft_list,
                                  const environment::SimulationTime& simulation_time) {
  if (!is_enabled_) return;

  if (simulators_.empty()) {
    throw std::runtime_error("Skydel HIL is not started. Call StreamInitialSamples first.");
  }
  try {
    StreamSamples(spacecraft_list, simulation_time, false, true);
  } catch (...) {
    Close();
    throw;
  }
}

void SkydelHil::Close() {
  if (!simulators_.empty() && simulators_.front()) {
    try {
      simulators_.front()->stop();
    } catch (const std::exception& e) {
      std::cerr << "Warning: Failed to stop Skydel simulation: " << e.what() << std::endl;
    } catch (...) {
      std::cerr << "Warning: Failed to stop Skydel simulation: unknown exception" << std::endl;
    }
  }
  for (auto& simulator : simulators_) {
    if (!simulator) continue;
    try {
      simulator->disconnect();
    } catch (const std::exception& e) {
      std::cerr << "Warning: Failed to disconnect from Skydel instance: " << e.what() << std::endl;
    } catch (...) {
      std::cerr << "Warning: Failed to disconnect from Skydel instance: unknown exception" << std::endl;
    }
  }

  simulators_.clear();
  spacecraft_ids_.clear();
  instance_ids_.clear();
  config_paths_.clear();
  simulation_start_timestamp_ms_ = 0.0;
  last_streamed_elapsed_ms_ = -1;
  next_warning_elapsed_ms_ = 0;
  is_enabled_ = false;
}

void SkydelHil::LoadConfiguration(const std::string& base_ini_path, const unsigned int number_of_spacecraft) {
  s2e::setting_file_reader::IniAccess base_ini(base_ini_path);
  const std::string hil_ini_path = base_ini.ReadString("SIMULATION_SETTINGS", "skydel_hil_file");
  if (hil_ini_path.empty() || hil_ini_path == "NULL") {
    is_enabled_ = false;
    return;
  }

  s2e::setting_file_reader::IniAccess hil_ini(hil_ini_path);
  is_enabled_ = hil_ini.ReadEnable("SKYDEL_HIL", "enable");
  if (!is_enabled_) return;

  skydel_host_ = hil_ini.ReadString("SKYDEL_HIL", "skydel_host");
  enable_log_raw_ = hil_ini.ReadEnable("SKYDEL_HIL", "enable_log_raw");
  raw_rate_hz_ = hil_ini.ReadInt("SKYDEL_HIL", "raw_rate_hz");
  enable_log_hil_input_ = hil_ini.ReadEnable("SKYDEL_HIL", "enable_log_hil_input");
  enable_hil_streaming_check_ = hil_ini.ReadEnable("SKYDEL_HIL", "enable_hil_streaming_check");
  output_period_ms_ = hil_ini.ReadInt("SKYDEL_HIL", "output_period_ms");
  sync_duration_ms_ = hil_ini.ReadInt("SKYDEL_HIL", "sync_duration_ms");
  hil_tjoin_ms_ = hil_ini.ReadInt("SKYDEL_HIL", "hil_tjoin_ms");
  sync_port_ = hil_ini.ReadInt("SKYDEL_HIL", "sync_port");
  warning_check_period_ms_ = hil_ini.ReadInt("SKYDEL_HIL", "warning_check_period_ms");
  const int number_of_vehicles = hil_ini.ReadInt("SKYDEL_HIL", "number_of_vehicles");

  if (output_period_ms_ <= 0 || sync_duration_ms_ <= 0 || hil_tjoin_ms_ <= 0 || sync_port_ <= 0 || warning_check_period_ms_ <= 0) {
    throw std::runtime_error("Skydel HIL timing and synchronization parameters must be positive");
  }
  if (raw_rate_hz_ != 10 && raw_rate_hz_ != 100 && raw_rate_hz_ != 1000) {
    throw std::runtime_error("Skydel HIL raw_rate_hz must be 10, 100, or 1000");
  }
  if (number_of_vehicles <= 0) {
    throw std::runtime_error("Skydel HIL requires at least one vehicle");
  }
  next_warning_elapsed_ms_ = warning_check_period_ms_;

  spacecraft_ids_.reserve(static_cast<size_t>(number_of_vehicles));
  instance_ids_.reserve(static_cast<size_t>(number_of_vehicles));
  config_paths_.reserve(static_cast<size_t>(number_of_vehicles));

  for (int i = 0; i < number_of_vehicles; ++i) {
    const std::string section = "VEHICLE_" + std::to_string(i);
    const int spacecraft_id = hil_ini.ReadInt(section.c_str(), "spacecraft_id");
    const int instance_id = hil_ini.ReadInt(section.c_str(), "instance");
    if (spacecraft_id < 0 || instance_id < 0) {
      throw std::runtime_error("Skydel HIL spacecraft_id and instance must be non-negative");
    }
    spacecraft_ids_.push_back(static_cast<unsigned int>(spacecraft_id));
    instance_ids_.push_back(static_cast<unsigned int>(instance_id));
    config_paths_.push_back(hil_ini.ReadString(section.c_str(), "config_path"));
  }

  ValidateVehicleConfigs(number_of_spacecraft);
}

void SkydelHil::ValidateVehicleConfigs(const unsigned int number_of_spacecraft) const {
  if (skydel_host_.empty() || skydel_host_ == "NULL") {
    throw std::runtime_error("Skydel HIL host must not be empty");
  }

  std::set<unsigned int> spacecraft_ids;
  std::set<unsigned int> instance_ids;
  for (size_t index = 0; index < spacecraft_ids_.size(); ++index) {
    if (spacecraft_ids_[index] >= number_of_spacecraft) {
      throw std::runtime_error("Skydel HIL spacecraft_id exceeds the number of simulated spacecraft");
    }
    if (config_paths_[index].empty() || config_paths_[index] == "NULL") {
      throw std::runtime_error("Skydel HIL config_path must not be empty");
    }
    if (!spacecraft_ids.insert(spacecraft_ids_[index]).second) {
      throw std::runtime_error("Skydel HIL spacecraft_id must be unique");
    }
    if (!instance_ids.insert(instance_ids_[index]).second) {
      throw std::runtime_error("Skydel HIL instance must be unique");
    }
  }
}

void SkydelHil::SetupSimulators(const environment::SimulationTime& simulation_time) {
  if (!simulators_.empty()) return;

  const double start_second = simulation_time.GetStartSecond();
  if (start_second != std::floor(start_second)) {
    throw std::runtime_error("Skydel HIL requires an integer-second simulation start time");
  }

  const s2e::time_system::DateTime utc_start_time(
      static_cast<size_t>(simulation_time.GetStartYear()), static_cast<size_t>(simulation_time.GetStartMonth()),
      static_cast<size_t>(simulation_time.GetStartDay()), static_cast<size_t>(simulation_time.GetStartHour()),
      static_cast<size_t>(simulation_time.GetStartMinute()), start_second);
  const s2e::time_system::DateTime gps_start_time(
      s2e::time_system::EpochTime(utc_start_time) + s2e::time_system::GpsTime::GetLeapSecondAheadFromUtc());
  const Sdx::DateTime start_time(static_cast<int>(gps_start_time.GetYear()), static_cast<int>(gps_start_time.GetMonth()),
                                 static_cast<int>(gps_start_time.GetDay()), static_cast<int>(gps_start_time.GetHour()),
                                 static_cast<int>(gps_start_time.GetMinute()), static_cast<int>(gps_start_time.GetSecond()));
  const int duration_sec = std::max(1, static_cast<int>(std::ceil(simulation_time.GetEndTime_s())));

  simulators_.reserve(instance_ids_.size());
  for (size_t index = 0; index < instance_ids_.size(); ++index) {
    auto simulator = std::make_shared<Sdx::RemoteSimulator>();
    simulator->setVerbose(false);
    if (!simulator->connect(skydel_host_, static_cast<int>(instance_ids_[index]))) {
      throw std::runtime_error("Failed to connect to Skydel instance " + std::to_string(instance_ids_[index]));
    }

    simulator->call(Sdx::Cmd::Open::create(config_paths_[index], true));
    simulator->call(Sdx::Cmd::SetVehicleTrajectory::create("HIL"));
    simulator->call(Sdx::Cmd::SetHilTjoin::create(hil_tjoin_ms_));
    simulator->call(Sdx::Cmd::SetStartTimeMode::create("Custom"));
    simulator->call(Sdx::Cmd::SetGpsStartTime::create(start_time));
    simulator->call(Sdx::Cmd::SetDuration::create(duration_sec));
    simulator->call(Sdx::Cmd::EnableLogRaw::create(enable_log_raw_));
    simulator->call(Sdx::Cmd::SetLogRawRate::create(raw_rate_hz_));
    simulator->call(Sdx::Cmd::EnableLogHILInput::create(enable_log_hil_input_));

    simulator->setHilStreamingCheckEnabled(enable_hil_streaming_check_);
    simulators_.push_back(std::move(simulator));
  }

  Sdx::RemoteSimulator& main = *simulators_.front();
  if (simulators_.size() > 1) {
    main.call(Sdx::Cmd::SetSyncServer::create(sync_port_));
  }
  main.call(Sdx::Cmd::EnableMainInstanceSync::create(true));

  for (size_t index = 1; index < simulators_.size(); ++index) {
    simulators_[index]->call(Sdx::Cmd::SetSyncClient::create(skydel_host_, sync_port_));
    simulators_[index]->call(Sdx::Cmd::EnableWorkerInstanceSync::create(true));
  }

  if (simulators_.size() > 1) {
    auto status = Sdx::Cmd::GetMainInstanceStatusResult::dynamicCast(main.call(Sdx::Cmd::GetMainInstanceStatus::create()));
    if (!status->isMainInstance() || status->workerInstanceConnected() != static_cast<int>(simulators_.size() - 1)) {
      Sdx::preciseSleepUntilMs(Sdx::getCurrentTimeMs() + 1000.0);
      status = Sdx::Cmd::GetMainInstanceStatusResult::dynamicCast(main.call(Sdx::Cmd::GetMainInstanceStatus::create()));
    }
    if (!status->isMainInstance() || status->workerInstanceConnected() != static_cast<int>(simulators_.size() - 1)) {
      throw std::runtime_error("Skydel worker instances did not connect to the main instance");
    }
  }

  main.call(Sdx::Cmd::ArmPPS::create());
  main.call(Sdx::Cmd::WaitAndResetPPS::create());
  main.call(Sdx::Cmd::StartPPS::create(sync_duration_ms_));

  const auto pps0 = Sdx::Cmd::GetComputerSystemTimeSinceEpochAtPps0Result::dynamicCast(
      main.call(Sdx::Cmd::GetComputerSystemTimeSinceEpochAtPps0::create()));
  simulation_start_timestamp_ms_ = pps0->milliseconds() + static_cast<double>(sync_duration_ms_);
}

void SkydelHil::StreamSamples(const std::vector<const spacecraft::Spacecraft*>& spacecraft_list,
                              const environment::SimulationTime& simulation_time, const bool force, const bool wait_for_timestamp) {
  const int64_t elapsed_time_ms = static_cast<int64_t>(std::llround(simulation_time.GetElapsedTime_s() * 1000.0));
  if (!force && last_streamed_elapsed_ms_ >= 0) {
    if (elapsed_time_ms <= last_streamed_elapsed_ms_) return;
    if (elapsed_time_ms - last_streamed_elapsed_ms_ < output_period_ms_) return;
  }

  if (wait_for_timestamp) {
    Sdx::preciseSleepUntilMs(simulation_start_timestamp_ms_ + static_cast<double>(elapsed_time_ms));
  }

  for (size_t index = 0; index < spacecraft_ids_.size(); ++index) {
    if (spacecraft_ids_[index] >= spacecraft_list.size() || spacecraft_list[spacecraft_ids_[index]] == nullptr) {
      throw std::runtime_error("Invalid spacecraft_id in Skydel HIL configuration");
    }
    PushSample(index, *spacecraft_list[spacecraft_ids_[index]], elapsed_time_ms);
  }

  if (elapsed_time_ms >= next_warning_elapsed_ms_) {
    for (auto& simulator : simulators_) {
      DisplayHilExtrapolationWarnings(*simulator);
    }
    next_warning_elapsed_ms_ = elapsed_time_ms + warning_check_period_ms_;
  }

  last_streamed_elapsed_ms_ = elapsed_time_ms;
}

void SkydelHil::PushSample(const size_t index, const spacecraft::Spacecraft& spacecraft, const int64_t elapsed_time_ms) {
  const auto& dynamics = spacecraft.GetDynamics();
  const auto& orbit = dynamics.GetOrbit();
  const auto& celestial_information = spacecraft.GetLocalEnvironment().GetCelestialInformation().GetGlobalInformation();

  const auto position_ecef_m = orbit.GetPosition_ecef_m();
  const auto velocity_ecef_m_s = orbit.GetVelocity_ecef_m_s();
  const auto acceleration_ecef_m_s2 = CalcTotalAccelerationEcef_m_s2(spacecraft);

  const auto dcm_i_to_ecef = celestial_information.GetEarthRotation().GetDcmJ2000ToEcef();
  const auto dcm_ecef_to_ned = CalcDcmEcefToNed(orbit.GetGeodeticPosition());
  const auto dcm_ned_to_i = dcm_i_to_ecef.Transpose() * dcm_ecef_to_ned.Transpose();
  const auto dcm_ned_to_b = dynamics.GetAttitude().GetQuaternion_i2b().ConvertToDcm() * dcm_ned_to_i;
  const auto roll_pitch_yaw_rad = s2e::math::Quaternion::ConvertFromDcm(dcm_ned_to_b).Normalize().ConvertToEuler();

  const auto angular_velocity_b_rad_s = dynamics.GetAttitude().GetAngularVelocity_b_rad_s();
  const auto angular_acceleration_b_rad_s2 = dynamics.GetAttitude().GetAngularAcceleration_b_rad_s2();

  const Sdx::Ecef position(position_ecef_m[0], position_ecef_m[1], position_ecef_m[2]);
  const Sdx::Attitude attitude(roll_pitch_yaw_rad[2], roll_pitch_yaw_rad[1], roll_pitch_yaw_rad[0]);
  const Sdx::Ecef velocity(velocity_ecef_m_s[0], velocity_ecef_m_s[1], velocity_ecef_m_s[2]);
  const Sdx::Attitude angular_velocity(angular_velocity_b_rad_s[2], angular_velocity_b_rad_s[1], angular_velocity_b_rad_s[0]);
  const Sdx::Ecef acceleration(acceleration_ecef_m_s2[0], acceleration_ecef_m_s2[1], acceleration_ecef_m_s2[2]);
  const Sdx::Attitude angular_acceleration(angular_acceleration_b_rad_s2[2], angular_acceleration_b_rad_s2[1],
                                           angular_acceleration_b_rad_s2[0]);

  if (!simulators_[index]->pushEcefNed(static_cast<double>(elapsed_time_ms), position, attitude, velocity, angular_velocity, acceleration,
                                       angular_acceleration)) {
    throw std::runtime_error("Failed to push HIL state to Skydel instance " + std::to_string(instance_ids_[index]));
  }
}

math::Matrix<3, 3> SkydelHil::CalcDcmEcefToNed(const geodesy::GeodeticPosition& geodetic_position) const {
  // S2E local topographic frame is ENU, while Skydel pushEcefNed uses NED.
  const auto dcm_ecef_to_enu = geodetic_position.GetQuaternionXcxfToLtc().ConvertToDcm();
  math::Matrix<3, 3> dcm_ecef_to_ned;
  for (size_t column = 0; column < 3; ++column) {
    dcm_ecef_to_ned[0][column] = dcm_ecef_to_enu[1][column];
    dcm_ecef_to_ned[1][column] = dcm_ecef_to_enu[0][column];
    dcm_ecef_to_ned[2][column] = -dcm_ecef_to_enu[2][column];
  }
  return dcm_ecef_to_ned;
}

math::Vector<3> SkydelHil::CalcTotalAccelerationEcef_m_s2(const spacecraft::Spacecraft& spacecraft) const {
  const auto& dynamics = spacecraft.GetDynamics();
  const auto& orbit = dynamics.GetOrbit();
  const auto& celestial_information = spacecraft.GetLocalEnvironment().GetCelestialInformation().GetGlobalInformation();
  auto& disturbances = const_cast<s2e::disturbances::Disturbances&>(spacecraft.GetDisturbances());
  auto& components = const_cast<s2e::spacecraft::InstalledComponents&>(spacecraft.GetInstalledComponents());

  math::Vector<3> acceleration_i_m_s2 = disturbances.GetAcceleration_i_m_s2();

  // Add the acceleration due to disturbances and installed components
  const math::Vector<3> force_b_N = disturbances.GetForce_b_N() + components.GenerateForce_b_N();
  const double mass_kg = spacecraft.GetStructure().GetKinematicsParameters().GetMass_kg();
  const math::Vector<3> force_i_N = dynamics.GetAttitude().GetQuaternion_i2b().InverseFrameConversion(force_b_N);
  acceleration_i_m_s2 += (1.0 / mass_kg) * force_i_N;

  // Add the central gravity acceleration
  const math::Vector<3> position_i_m = orbit.GetPosition_i_m();
  const double radius_m = position_i_m.CalcNorm();
  const double mu_m3_s2 = celestial_information.GetCenterBodyGravityConstant_m3_s2();
  acceleration_i_m_s2 -= mu_m3_s2 / (radius_m * radius_m * radius_m) * position_i_m;

  // Convert the acceleration from the inertial frame to the ECEF frame.
  const auto& earth_rotation = celestial_information.GetEarthRotation();
  const auto dcm_i_to_ecef = earth_rotation.GetDcmJ2000ToEcef();
  const auto dcm_dot_i_to_ecef = earth_rotation.GetDcmJ2000ToEcefDerivative();
  const auto dcm_ddot_i_to_ecef = earth_rotation.GetDcmJ2000ToEcefSecondDerivative();
  const math::Vector<3> velocity_i_m_s = orbit.GetVelocity_i_m_s();

  math::Vector<3> acceleration_ecef_m_s2 = dcm_i_to_ecef * acceleration_i_m_s2;
  acceleration_ecef_m_s2 += 2.0 * (dcm_dot_i_to_ecef * velocity_i_m_s);
  acceleration_ecef_m_s2 += dcm_ddot_i_to_ecef * position_i_m;
  return acceleration_ecef_m_s2;
}

void SkydelHil::DisplayHilExtrapolationWarnings(Sdx::RemoteSimulator& simulator) const {
  const bool is_verbose = simulator.isVerbose();
  simulator.setVerbose(false);
  const auto result =
      Sdx::Cmd::GetHilExtrapolationStateResult::dynamicCast(simulator.call(Sdx::Cmd::GetHilExtrapolationState::create()));
  if (result->state() == Sdx::HilExtrapolationState::NonDeterministic) {
    std::cout << "Warning: HIL non deterministic extrapolation at millisecond " << result->elapsedTime() << std::endl;
  } else if (result->state() == Sdx::HilExtrapolationState::Snap) {
    std::cout << "Warning: HIL position snap at millisecond " << result->elapsedTime() << std::endl;
  }
  simulator.setVerbose(is_verbose);
}

}  // namespace s2e::simulation
