/*
 * @file orbit_observer.cpp
 * @brief Ideal component which can observe orbit
 */

#include "orbit_observer.hpp"

#include <library/initialize/initialize_file_access.hpp>

OrbitObserver::OrbitObserver(const int prescaler, ClockGenerator* clock_generator, const ErrorFrame error_frame,
                             const libra::Vector<6> error_standard_deviation, const Orbit& orbit)
    : Component(prescaler, clock_generator), orbit_(orbit) {
  for (size_t i = 0; i < 6; i++) {
    normal_random_noise_[i].SetParameters(0.0, error_standard_deviation[i]);
  }
}

void OrbitObserver::MainRoutine(const int time_count) {
  UNUSED(time_count);

  libra::Quaternion q_i2rtn = orbit_.CalcQuaternion_i2lvlh();
  switch (error_frame_) {
    case ErrorFrame::kInertial:
      observed_position_i_m_ = AddPositionNoise(orbit_.GetPosition_i_m());
      // Frame conversion
      observed_position_rtn_m_ = q_i2rtn.FrameConversion(observed_position_i_m_);
      break;
    case ErrorFrame::kRtn:
      // observed_position_rtn_m_ = AddPositionNoise(orbit_.GetPosition_);
      //  Frame conversion
      observed_position_i_m_ = q_i2rtn.InverseFrameConversion(observed_position_rtn_m_);
      break;
    default:
      break;
  }
}

std::string OrbitObserver::GetLogHeader() const {
  std::string str_tmp = "";

  std::string head = "orbit_observer_";
  str_tmp += WriteVector(head + "position", "i", "m", 3);
  str_tmp += WriteVector(head + "velocity", "i", "m/s", 3);

  return str_tmp;
}

std::string OrbitObserver::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteVector(observed_position_i_m_, 16);
  str_tmp += WriteVector(observed_velocity_i_m_s_, 16);

  return str_tmp;
}

void AddNoise() {}

OrbitObserver InitializeOrbitObserver(ClockGenerator* clock_generator, const std::string file_name, const Orbit& orbit) {
  // General
  IniAccess ini_file(file_name);

  // CompoBase
  int prescaler = ini_file.ReadInt("COMPONENT_BASE", "prescaler");
  if (prescaler <= 1) prescaler = 1;

  // Noise
  const ErrorFrame error_frame = ErrorFrame::kInertial;
  const libra::Vector<6> error_standard_deviation;

  OrbitObserver orbit_observer(prescaler, clock_generator, error_frame, error_standard_deviation, orbit);

  return orbit_observer;
}
