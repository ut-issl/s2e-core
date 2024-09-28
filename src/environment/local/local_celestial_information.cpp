/**
 * @file local_celestial_information.cpp
 * @brief Class to manage celestial body information in the spacecraft body frame
 */

#include "local_celestial_information.hpp"

#include <SpiceUsr.h>

#include <algorithm>
#include <iostream>
#include <locale>
#include <sstream>

#include "logger/log_utility.hpp"

namespace s2e::environment {

LocalCelestialInformation::LocalCelestialInformation(const CelestialInformation* global_celestial_information)
    : global_celestial_information_(global_celestial_information) {
  int num_of_state = global_celestial_information_->GetNumberOfSelectedBodies() * 3;
  celestial_body_position_from_center_b_m_ = new double[num_of_state];
  celestial_body_velocity_from_center_b_m_s_ = new double[num_of_state];
  celestial_body_position_from_spacecraft_i_m_ = new double[num_of_state];
  celestial_body_velocity_from_spacecraft_i_m_s_ = new double[num_of_state];
  celestial_body_position_from_spacecraft_b_m_ = new double[num_of_state];
  celestial_body_velocity_from_spacecraft_b_m_s_ = new double[num_of_state];

  for (int i = 0; i < num_of_state; i++) {
    celestial_body_position_from_center_b_m_[i] = 0.0;
    celestial_body_velocity_from_center_b_m_s_[i] = 0.0;
    celestial_body_position_from_spacecraft_i_m_[i] = 0.0;
    celestial_body_velocity_from_spacecraft_i_m_s_[i] = 0.0;
    celestial_body_position_from_spacecraft_b_m_[i] = 0.0;
    celestial_body_velocity_from_spacecraft_b_m_s_[i] = 0.0;
  }
}

LocalCelestialInformation::~LocalCelestialInformation() {
  delete[] celestial_body_position_from_center_b_m_;
  delete[] celestial_body_velocity_from_center_b_m_s_;
  delete[] celestial_body_position_from_spacecraft_i_m_;
  delete[] celestial_body_velocity_from_spacecraft_i_m_s_;
  delete[] celestial_body_position_from_spacecraft_b_m_;
  delete[] celestial_body_velocity_from_spacecraft_b_m_s_;
}

void LocalCelestialInformation::UpdateAllObjectsInformation(const s2e::math::Vector<3> spacecraft_position_from_center_i_m,
                                                            const s2e::math::Vector<3> spacecraft_velocity_from_center_i_m_s,
                                                            const s2e::math::Quaternion quaternion_i2b,
                                                            const s2e::math::Vector<3> spacecraft_angular_velocity_rad_s) {
  s2e::math::Vector<3> celestial_body_position_i_m, celestial_body_velocity_i_m_s;
  for (int i = 0; i < global_celestial_information_->GetNumberOfSelectedBodies(); i++) {
    celestial_body_position_i_m = global_celestial_information_->GetPositionFromCenter_i_m(i);
    celestial_body_velocity_i_m_s = global_celestial_information_->GetVelocityFromCenter_i_m_s(i);
    // Change origin of frame
    for (int j = 0; j < 3; j++) {
      celestial_body_position_from_spacecraft_i_m_[i * 3 + j] = celestial_body_position_i_m[j] - spacecraft_position_from_center_i_m[j];
      celestial_body_velocity_from_spacecraft_i_m_s_[i * 3 + j] = celestial_body_velocity_i_m_s[j] - spacecraft_velocity_from_center_i_m_s[j];
    }
  }
  CalcAllPosVel_b(quaternion_i2b, spacecraft_angular_velocity_rad_s);

  return;
}

void LocalCelestialInformation::CalcAllPosVel_b(const s2e::math::Quaternion quaternion_i2b, const s2e::math::Vector<3> spacecraft_angular_velocity_rad_s) {
  s2e::math::Vector<3> celestial_body_position_i_m, celestial_body_velocity_i_m_s;
  double r_buf1_i[3], velocity_buf1_i[3], r_buf1_b[3], velocity_buf1_b[3];
  double r_buf2_i[3], velocity_buf2_i[3], r_buf2_b[3], velocity_buf2_b[3];
  for (int i = 0; i < global_celestial_information_->GetNumberOfSelectedBodies(); i++) {
    celestial_body_position_i_m = global_celestial_information_->GetPositionFromCenter_i_m(i);
    celestial_body_velocity_i_m_s = global_celestial_information_->GetVelocityFromCenter_i_m_s(i);
    for (int j = 0; j < 3; j++) {
      r_buf1_i[j] = celestial_body_position_i_m[j];
      r_buf2_i[j] = celestial_body_position_from_spacecraft_i_m_[i * 3 + j];
      velocity_buf1_i[j] = celestial_body_velocity_i_m_s[j];
      velocity_buf2_i[j] = celestial_body_velocity_from_spacecraft_i_m_s_[i * 3 + j];
    }
    ConvertInertialToBody(r_buf1_i, r_buf1_b, quaternion_i2b);
    ConvertInertialToBody(r_buf2_i, r_buf2_b, quaternion_i2b);
    ConvertVelocityInertialToBody(r_buf1_i, velocity_buf1_i, velocity_buf1_b, quaternion_i2b, spacecraft_angular_velocity_rad_s);
    ConvertVelocityInertialToBody(r_buf2_i, velocity_buf2_i, velocity_buf2_b, quaternion_i2b, spacecraft_angular_velocity_rad_s);

    for (int j = 0; j < 3; j++) {
      celestial_body_position_from_center_b_m_[i * 3 + j] = r_buf1_b[j];
      celestial_body_position_from_spacecraft_b_m_[i * 3 + j] = r_buf2_b[j];
      celestial_body_velocity_from_center_b_m_s_[i * 3 + j] = velocity_buf1_b[j];
      celestial_body_velocity_from_spacecraft_b_m_s_[i * 3 + j] = velocity_buf2_b[j];
    }
  }
}

void LocalCelestialInformation::ConvertInertialToBody(const double* input_i, double* output_b, s2e::math::Quaternion quaternion_i2b) {
  s2e::math::Vector<3> temp_i;
  for (int i = 0; i < 3; i++) {
    temp_i[i] = input_i[i];
  }
  s2e::math::Vector<3> temp_b = quaternion_i2b.FrameConversion(temp_i);
  for (int i = 0; i < 3; i++) {
    output_b[i] = temp_b[i];
  }
}

void LocalCelestialInformation::ConvertVelocityInertialToBody(const double* position_i, const double* velocity_i, double* velocity_b,
                                                              const s2e::math::Quaternion quaternion_i2b, const s2e::math::Vector<3> angular_velocity_b) {
  // copy input vector
  s2e::math::Vector<3> vi;
  for (int i = 0; i < 3; i++) {
    vi[i] = velocity_i[i];
  }
  s2e::math::Vector<3> ri;
  for (int i = 0; i < 3; i++) {
    ri[i] = position_i[i];
  }

  // convert body rate vector into that in inertial coordinate
  s2e::math::Vector<3> wb;
  for (int i = 0; i < 3; i++) {
    wb[i] = angular_velocity_b[i];
  }

  // compute cross term wxr
  s2e::math::Vector<3> wxposition_i = OuterProduct(wb, ri);
  // compute dr/dt + wxr
  for (int i = 0; i < 3; i++) {
    vi[i] = vi[i] - wxposition_i[i];
  }
  // convert vector in inertial coordinate into that in body coordinate
  s2e::math::Vector<3> temp_b = quaternion_i2b.FrameConversion(vi);
  for (int i = 0; i < 3; i++) {
    velocity_b[i] = temp_b[i];
  }
}

s2e::math::Vector<3> LocalCelestialInformation::GetPositionFromSpacecraft_i_m(const char* body_name) const {
  s2e::math::Vector<3> position;
  int index = 0;
  index = global_celestial_information_->CalcBodyIdFromName(body_name);
  for (int i = 0; i < 3; i++) {
    position[i] = celestial_body_position_from_spacecraft_i_m_[index * 3 + i];
  }
  return position;
}

s2e::math::Vector<3> LocalCelestialInformation::GetCenterBodyPositionFromSpacecraft_i_m() const {
  std::string body_name = global_celestial_information_->GetCenterBodyName();
  s2e::math::Vector<3> position = GetPositionFromSpacecraft_i_m(body_name.c_str());
  return position;
}

s2e::math::Vector<3> LocalCelestialInformation::GetPositionFromSpacecraft_b_m(const char* body_name) const {
  s2e::math::Vector<3> position;
  int index = 0;
  index = global_celestial_information_->CalcBodyIdFromName(body_name);
  for (int i = 0; i < 3; i++) {
    position[i] = celestial_body_position_from_spacecraft_b_m_[index * 3 + i];
  }
  return position;
}

s2e::math::Vector<3> LocalCelestialInformation::GetCenterBodyPositionFromSpacecraft_b_m(void) const {
  std::string body_name = global_celestial_information_->GetCenterBodyName();
  s2e::math::Vector<3> position = GetPositionFromSpacecraft_b_m(body_name.c_str());
  return position;
}

std::string LocalCelestialInformation::GetLogHeader() const {
  SpiceBoolean found;
  const int maxlen = 100;
  char namebuf[maxlen];
  std::string str_tmp = "";
  for (int i = 0; i < global_celestial_information_->GetNumberOfSelectedBodies(); i++) {
    SpiceInt planet_id = global_celestial_information_->GetSelectedBodyIds()[i];
    // Acquisition of body name from id
    bodc2n_c(planet_id, maxlen, namebuf, (SpiceBoolean*)&found);
    std::string name = namebuf;

    std::locale loc = std::locale::classic();
    std::transform(name.begin(), name.end(), name.begin(), [loc](char c) { return std::tolower(c, loc); });

    std::string body_pos = name + "_position_from_spacecraft";
    std::string body_vel = name + "_velocity_from_spacecraft";
    str_tmp += logger::WriteVector(body_pos, "b", "m", 3);
    str_tmp += logger::WriteVector(body_vel, "b", "m/s", 3);
  }
  return str_tmp;
}

std::string LocalCelestialInformation::GetLogValue() const {
  std::string str_tmp = "";
  for (int i = 0; i < global_celestial_information_->GetNumberOfSelectedBodies(); i++) {
    for (int j = 0; j < 3; j++) {
      str_tmp += WriteScalar(celestial_body_position_from_spacecraft_b_m_[i * 3 + j]);
    }
    for (int j = 0; j < 3; j++) {
      str_tmp += WriteScalar(celestial_body_velocity_from_spacecraft_b_m_s_[i * 3 + j]);
    }
  }
  return str_tmp;
}

} // namespace s2e::environment
