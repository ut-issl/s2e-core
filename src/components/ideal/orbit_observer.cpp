/*
 * @file orbit_observer.cpp
 * @brief Ideal component which can observe orbit
 */

#include "orbit_observer.hpp"

#include <library/initialize/initialize_file_access.hpp>
#include <library/randomization/global_randomization.hpp>

OrbitObserver::OrbitObserver(const int prescaler, ClockGenerator* clock_generator, const ErrorFrame error_frame,
                             const libra::Vector<6> error_standard_deviation, const Orbit& orbit)
    : Component(prescaler, clock_generator), error_frame_(error_frame), orbit_(orbit) {
  for (size_t i = 0; i < 6; i++) {
    normal_random_noise_[i].SetParameters(0.0, error_standard_deviation[i], global_randomization.MakeSeed());
  }
}

void OrbitObserver::MainRoutine(const int time_count) {
  UNUSED(time_count);

  // Calc noise
  libra::Vector<3> position_error_i_m{0.0};
  libra::Vector<3> position_error_rtn_m{0.0};
  libra::Quaternion q_i2rtn = orbit_.CalcQuaternion_i2lvlh();
  switch (error_frame_) {
    case ErrorFrame::kInertial:
      for (size_t axis = 0; axis < 3; axis++) {
        position_error_i_m[axis] = normal_random_noise_[axis];
      }
      break;
    case ErrorFrame::kRtn:
      for (size_t axis = 0; axis < 3; axis++) {
        position_error_rtn_m[axis] = normal_random_noise_[axis];
      }
      //  Frame conversion
      position_error_i_m = q_i2rtn.InverseFrameConversion(position_error_rtn_m);
      break;
    default:
      break;
  }

  // Get observed value
  observed_position_i_m_ = orbit_.GetPosition_i_m() + position_error_i_m;
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
