/**
 * @file local_celestial_information.cpp
 * @brief Class to manage celestial body information in the spacecraft body frame
 */

#include "local_celestial_information.hpp"

#include <SpiceUsr.h>

#include <algorithm>
#include <iostream>
#include <sstream>

#include "library/logger/log_utility.hpp"

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

void LocalCelestialInformation::UpdateAllObjectsInfo(const libra::Vector<3> sc_pos_from_center_i, const libra::Vector<3> sc_vel_from_center_i,
                                                     const libra::Quaternion q_i2b, const libra::Vector<3> sc_body_rate) {
  Vector<3> pos_center_i, vel_center_i;
  for (int i = 0; i < global_celestial_information_->GetNumberOfSelectedBodies(); i++) {
    pos_center_i = global_celestial_information_->GetPositionFromCenter_i_m(i);
    vel_center_i = global_celestial_information_->GetVelocityFromCenter_i_m_s(i);
    // Change origin of frame
    for (int j = 0; j < 3; j++) {
      celestial_body_position_from_spacecraft_i_m_[i * 3 + j] = pos_center_i[j] - sc_pos_from_center_i[j];
      celestial_body_velocity_from_spacecraft_i_m_s_[i * 3 + j] = vel_center_i[j] - sc_vel_from_center_i[j];
    }
  }
  CalcAllPosVel_b(q_i2b, sc_body_rate);

  return;
}

void LocalCelestialInformation::CalcAllPosVel_b(const libra::Quaternion q_i2b, const libra::Vector<3> sc_body_rate) {
  libra::Vector<3> pos_center_i, vel_center_i;
  double r_buf1_i[3], v_buf1_i[3], r_buf1_b[3], v_buf1_b[3];
  double r_buf2_i[3], v_buf2_i[3], r_buf2_b[3], v_buf2_b[3];
  for (int i = 0; i < global_celestial_information_->GetNumberOfSelectedBodies(); i++) {
    pos_center_i = global_celestial_information_->GetPositionFromCenter_i_m(i);
    vel_center_i = global_celestial_information_->GetVelocityFromCenter_i_m_s(i);
    for (int j = 0; j < 3; j++) {
      r_buf1_i[j] = pos_center_i[j];
      r_buf2_i[j] = celestial_body_position_from_spacecraft_i_m_[i * 3 + j];
      v_buf1_i[j] = vel_center_i[j];
      v_buf2_i[j] = celestial_body_velocity_from_spacecraft_i_m_s_[i * 3 + j];
    }
    Convert_i2b(r_buf1_i, r_buf1_b, q_i2b);
    Convert_i2b(r_buf2_i, r_buf2_b, q_i2b);
    Convert_i2b_velocity(r_buf1_i, v_buf1_i, v_buf1_b, q_i2b, sc_body_rate);
    Convert_i2b_velocity(r_buf2_i, v_buf2_i, v_buf2_b, q_i2b, sc_body_rate);

    for (int j = 0; j < 3; j++) {
      celestial_body_position_from_center_b_m_[i * 3 + j] = r_buf1_b[j];
      celestial_body_position_from_spacecraft_b_m_[i * 3 + j] = r_buf2_b[j];
      celestial_body_velocity_from_center_b_m_s_[i * 3 + j] = v_buf1_b[j];
      celestial_body_velocity_from_spacecraft_b_m_s_[i * 3 + j] = v_buf2_b[j];
    }
  }
}

void LocalCelestialInformation::Convert_i2b(const double* src_i, double* dst_b, libra::Quaternion q_i2b) {
  libra::Vector<3> temp_i;
  for (int i = 0; i < 3; i++) {
    temp_i[i] = src_i[i];
  }
  libra::Vector<3> temp_b = q_i2b.frame_conv(temp_i);
  for (int i = 0; i < 3; i++) {
    dst_b[i] = temp_b[i];
  }
}

void LocalCelestialInformation::Convert_i2b_velocity(const double* r_i, const double* v_i, double* v_b, libra::Quaternion q_i2b,
                                                     const libra::Vector<3> bodyrate_b) {
  // copy input vector
  libra::Vector<3> vi;
  for (int i = 0; i < 3; i++) {
    vi[i] = v_i[i];
  }
  libra::Vector<3> ri;
  for (int i = 0; i < 3; i++) {
    ri[i] = r_i[i];
  }

  // convert body rate vector into that in inertial coordinate
  Vector<3> wb;
  for (int i = 0; i < 3; i++) {
    wb[i] = bodyrate_b[i];
  }

  // compute cross term wxr
  libra::Vector<3> wxr_i = outer_product(wb, ri);
  // compute dr/dt + wxr
  for (int i = 0; i < 3; i++) {
    vi[i] = vi[i] - wxr_i[i];
  }
  // convert vector in inertial coordinate into that in body coordinate
  libra::Vector<3> temp_b = q_i2b.frame_conv(vi);
  for (int i = 0; i < 3; i++) {
    v_b[i] = temp_b[i];
  }
}

libra::Vector<3> LocalCelestialInformation::GetPosFromSC_i(const char* body_name) const {
  libra::Vector<3> position;
  int index = 0;
  index = global_celestial_information_->CalcBodyIdFromName(body_name);
  for (int i = 0; i < 3; i++) {
    position[i] = celestial_body_position_from_spacecraft_i_m_[index * 3 + i];
  }
  return position;
}

Vector<3> LocalCelestialInformation::GetCenterBodyPosFromSC_i() const {
  std::string body_name = global_celestial_information_->GetCenterBodyName();
  Vector<3> position = GetPosFromSC_i(body_name.c_str());
  return position;
}

libra::Vector<3> LocalCelestialInformation::GetPosFromSC_b(const char* body_name) const {
  libra::Vector<3> position;
  int index = 0;
  index = global_celestial_information_->CalcBodyIdFromName(body_name);
  for (int i = 0; i < 3; i++) {
    position[i] = celestial_body_position_from_spacecraft_b_m_[index * 3 + i];
  }
  return position;
}

libra::Vector<3> LocalCelestialInformation::GetCenterBodyPosFromSC_b(void) const {
  std::string body_name = global_celestial_information_->GetCenterBodyName();
  libra::Vector<3> position = GetPosFromSC_b(body_name.c_str());
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
    std::transform(name.begin(), name.end(), name.begin(), ::tolower);
    std::string body_pos = name + "_position_from_spacecraft";
    std::string body_vel = name + "_velocity_from_spacecraft";
    str_tmp += WriteVector(body_pos, "b", "m", 3);
    str_tmp += WriteVector(body_vel, "b", "m/s", 3);
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
