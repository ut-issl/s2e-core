/*
 * @file orbit_observer.cpp
 * @brief Ideal component which can observe orbit
 */

#include "orbit_observer.hpp"

#include <math_physics/randomization/global_randomization.hpp>
#include <setting_file_reader/initialize_file_access.hpp>

OrbitObserver::OrbitObserver(const int prescaler, ClockGenerator* clock_generator, const NoiseFrame noise_frame,
                             const s2e::math::Vector<6> error_standard_deviation, const Orbit& orbit)
    : Component(prescaler, clock_generator), noise_frame_(noise_frame), orbit_(orbit) {
  for (size_t i = 0; i < 6; i++) {
    normal_random_noise_[i].SetParameters(0.0, error_standard_deviation[i], s2e::randomization::global_randomization.MakeSeed());
  }
}

void OrbitObserver::MainRoutine(const int time_count) {
  UNUSED(time_count);

  // Calc noise
  s2e::math::Vector<3> position_error_i_m{0.0};
  s2e::math::Vector<3> position_error_rtn_m{0.0};
  s2e::math::Vector<3> velocity_error_i_m_s{0.0};
  s2e::math::Vector<3> velocity_error_rtn_m_s{0.0};
  s2e::math::Quaternion q_i2rtn = orbit_.CalcQuaternion_i2lvlh();
  switch (noise_frame_) {
    case NoiseFrame::kInertial:
      for (size_t axis = 0; axis < 3; axis++) {
        position_error_i_m[axis] = normal_random_noise_[axis];
        velocity_error_i_m_s[axis] = normal_random_noise_[axis + 3];
      }
      break;
    case NoiseFrame::kRtn:
      for (size_t axis = 0; axis < 3; axis++) {
        position_error_rtn_m[axis] = normal_random_noise_[axis];
        velocity_error_rtn_m_s[axis] = normal_random_noise_[axis + 3];
      }
      // Frame conversion
      position_error_i_m = q_i2rtn.InverseFrameConversion(position_error_rtn_m);
      // For zero bias noise, we do not need to care the frame rotation effect.
      velocity_error_i_m_s = q_i2rtn.InverseFrameConversion(velocity_error_rtn_m_s);

      break;
    default:
      break;
  }

  // Get observed value
  observed_position_i_m_ = orbit_.GetPosition_i_m() + position_error_i_m;
  observed_velocity_i_m_s_ = orbit_.GetVelocity_i_m_s() + velocity_error_i_m_s;
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

NoiseFrame SetNoiseFrame(const std::string noise_frame) {
  if (noise_frame == "INERTIAL") {
    return NoiseFrame::kInertial;
  } else if (noise_frame == "RTN") {
    return NoiseFrame::kRtn;
  } else {
    std::cerr << "[WARNINGS] Orbit observer noise frame is not defined!" << std::endl;
    std::cerr << "The noise frame is automatically initialized as INERTIAL" << std::endl;
    return NoiseFrame::kInertial;
  }
}

OrbitObserver InitializeOrbitObserver(ClockGenerator* clock_generator, const std::string file_name, const Orbit& orbit) {
  // General
  IniAccess ini_file(file_name);

  // CompoBase
  int prescaler = ini_file.ReadInt("COMPONENT_BASE", "prescaler");
  if (prescaler <= 1) prescaler = 1;

  // Noise
  const NoiseFrame noise_frame = SetNoiseFrame(ini_file.ReadString("ORBIT_OBSERVER", "noise_frame"));
  s2e::math::Vector<6> noise_standard_deviation;
  ini_file.ReadVector("ORBIT_OBSERVER", "noise_standard_deviation", noise_standard_deviation);

  OrbitObserver orbit_observer(prescaler, clock_generator, noise_frame, noise_standard_deviation, orbit);

  return orbit_observer;
}
