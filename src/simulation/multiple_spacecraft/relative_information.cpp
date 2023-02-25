/**
 * @file relative_information.cpp
 * @brief Base class to manage relative information between spacecraft
 */

#include "relative_information.hpp"

RelativeInformation::RelativeInformation() {}

RelativeInformation::~RelativeInformation() {}

void RelativeInformation::Update() {
  for (size_t target_sat_id = 0; target_sat_id < dynamics_database_.size(); target_sat_id++) {
    for (size_t reference_sat_id = 0; reference_sat_id < dynamics_database_.size(); reference_sat_id++) {
      // Position
      libra::Vector<3> target_sat_pos_i = dynamics_database_.at(target_sat_id)->GetOrbit().GetPosition_i_m();
      libra::Vector<3> reference_sat_pos_i = dynamics_database_.at(reference_sat_id)->GetOrbit().GetPosition_i_m();
      rel_pos_list_i_m_[target_sat_id][reference_sat_id] = target_sat_pos_i - reference_sat_pos_i;
      rel_pos_list_rtn_m_[target_sat_id][reference_sat_id] = CalcRelativePosition_rtn_m(target_sat_id, reference_sat_id);

      // Distance
      rel_distance_list_m_[target_sat_id][reference_sat_id] = norm(rel_pos_list_i_m_[target_sat_id][reference_sat_id]);

      // Velocity
      libra::Vector<3> target_sat_vel_i = dynamics_database_.at(target_sat_id)->GetOrbit().GetVelocity_i_m_s();
      libra::Vector<3> reference_sat_vel_i = dynamics_database_.at(reference_sat_id)->GetOrbit().GetVelocity_i_m_s();
      rel_vel_list_i_m_s_[target_sat_id][reference_sat_id] = target_sat_vel_i - reference_sat_vel_i;
      rel_vel_list_rtn_m_s_[target_sat_id][reference_sat_id] = CalcRelativeVelocity_rtn_m_s(target_sat_id, reference_sat_id);

      // Attitude Quaternion
      rel_att_quaternion_list_[target_sat_id][reference_sat_id] = CalcRelativeAttitudeQuaternion(target_sat_id, reference_sat_id);
    }
  }
}

void RelativeInformation::RegisterDynamicsInfo(const int sat_id, const Dynamics* dynamics) {
  dynamics_database_.emplace(sat_id, dynamics);
  ResizeLists();
}

void RelativeInformation::RemoveDynamicsInfo(const int sat_id) {
  dynamics_database_.erase(sat_id);
  ResizeLists();
}

std::string RelativeInformation::GetLogHeader() const {
  std::string str_tmp = "";
  for (size_t target_sat_id = 0; target_sat_id < dynamics_database_.size(); target_sat_id++) {
    for (size_t reference_sat_id = 0; reference_sat_id < target_sat_id; reference_sat_id++) {
      str_tmp += WriteVector("sat" + std::to_string(target_sat_id) + " pos from sat" + std::to_string(reference_sat_id), "i", "m", 3);
    }
  }

  for (size_t target_sat_id = 0; target_sat_id < dynamics_database_.size(); target_sat_id++) {
    for (size_t reference_sat_id = 0; reference_sat_id < target_sat_id; reference_sat_id++) {
      str_tmp += WriteVector("sat" + std::to_string(target_sat_id) + " velocity from sat" + std::to_string(reference_sat_id), "i", "m/s", 3);
    }
  }

  for (size_t target_sat_id = 0; target_sat_id < dynamics_database_.size(); target_sat_id++) {
    for (size_t reference_sat_id = 0; reference_sat_id < target_sat_id; reference_sat_id++) {
      str_tmp += WriteVector("sat" + std::to_string(target_sat_id) + " pos from sat" + std::to_string(reference_sat_id), "rtn", "m", 3);
    }
  }

  for (size_t target_sat_id = 0; target_sat_id < dynamics_database_.size(); target_sat_id++) {
    for (size_t reference_sat_id = 0; reference_sat_id < target_sat_id; reference_sat_id++) {
      str_tmp += WriteVector("sat" + std::to_string(target_sat_id) + " velocity from sat" + std::to_string(reference_sat_id), "rtn", "m/s", 3);
    }
  }

  return str_tmp;
}

std::string RelativeInformation::GetLogValue() const {
  std::string str_tmp = "";
  for (size_t target_sat_id = 0; target_sat_id < dynamics_database_.size(); target_sat_id++) {
    for (size_t reference_sat_id = 0; reference_sat_id < target_sat_id; reference_sat_id++) {
      str_tmp += WriteVector(GetRelativePosition_i_m(target_sat_id, reference_sat_id));
    }
  }

  for (size_t target_sat_id = 0; target_sat_id < dynamics_database_.size(); target_sat_id++) {
    for (size_t reference_sat_id = 0; reference_sat_id < target_sat_id; reference_sat_id++) {
      str_tmp += WriteVector(GetRelativeVelocity_i_m_s(target_sat_id, reference_sat_id));
    }
  }

  for (size_t target_sat_id = 0; target_sat_id < dynamics_database_.size(); target_sat_id++) {
    for (size_t reference_sat_id = 0; reference_sat_id < target_sat_id; reference_sat_id++) {
      str_tmp += WriteVector(GetRelativePosition_rtn_m(target_sat_id, reference_sat_id));
    }
  }

  for (size_t target_sat_id = 0; target_sat_id < dynamics_database_.size(); target_sat_id++) {
    for (size_t reference_sat_id = 0; reference_sat_id < target_sat_id; reference_sat_id++) {
      str_tmp += WriteVector(GetRelativeVelocity_rtn_m_s(target_sat_id, reference_sat_id));
    }
  }

  return str_tmp;
}

void RelativeInformation::LogSetup(Logger& logger) { logger.AddLoggable(this); }

libra::Quaternion RelativeInformation::CalcRelativeAttitudeQuaternion(const int target_sat_id, const int reference_sat_id) {
  // Observer SC Body frame(obs_sat) -> ECI frame(i)
  Quaternion q_reference_i2b = dynamics_database_.at(reference_sat_id)->GetAttitude().GetQuaternion_i2b();
  Quaternion q_reference_b2i = q_reference_i2b.conjugate();

  // ECI frame(i) -> Target SC body frame(main_sat)
  Quaternion q_target_i2b = dynamics_database_.at(target_sat_id)->GetAttitude().GetQuaternion_i2b();

  return q_target_i2b * q_reference_b2i;
}

libra::Vector<3> RelativeInformation::CalcRelativePosition_rtn_m(const int target_sat_id, const int reference_sat_id) {
  libra::Vector<3> target_sat_pos_i = dynamics_database_.at(target_sat_id)->GetOrbit().GetPosition_i_m();
  libra::Vector<3> reference_sat_pos_i = dynamics_database_.at(reference_sat_id)->GetOrbit().GetPosition_i_m();
  libra::Vector<3> relative_pos_i = target_sat_pos_i - reference_sat_pos_i;

  // RTN frame for the reference satellite
  libra::Quaternion q_i2rtn = dynamics_database_.at(reference_sat_id)->GetOrbit().CalcQuaternion_i2lvlh();

  libra::Vector<3> relative_pos_rtn = q_i2rtn.frame_conv(relative_pos_i);
  return relative_pos_rtn;
}

libra::Vector<3> RelativeInformation::CalcRelativeVelocity_rtn_m_s(const int target_sat_id, const int reference_sat_id) {
  libra::Vector<3> target_sat_pos_i = dynamics_database_.at(target_sat_id)->GetOrbit().GetPosition_i_m();
  libra::Vector<3> reference_sat_pos_i = dynamics_database_.at(reference_sat_id)->GetOrbit().GetPosition_i_m();
  libra::Vector<3> relative_pos_i = target_sat_pos_i - reference_sat_pos_i;

  // RTN frame for the reference satellite
  libra::Quaternion q_i2rtn = dynamics_database_.at(reference_sat_id)->GetOrbit().CalcQuaternion_i2lvlh();

  // Rotation vector of RTN frame
  libra::Vector<3> reference_sat_vel_i = dynamics_database_.at(reference_sat_id)->GetOrbit().GetVelocity_i_m_s();
  libra::Vector<3> target_sat_vel_i = dynamics_database_.at(target_sat_id)->GetOrbit().GetVelocity_i_m_s();
  libra::Vector<3> rot_vec_rtn_i = cross(reference_sat_pos_i, reference_sat_vel_i);
  double r2_ref = norm(reference_sat_pos_i) * norm(reference_sat_pos_i);
  rot_vec_rtn_i /= r2_ref;
  libra::Vector<3> relative_vel_i = target_sat_vel_i - reference_sat_vel_i - cross(rot_vec_rtn_i, relative_pos_i);

  libra::Vector<3> relative_vel_rtn = q_i2rtn.frame_conv(relative_vel_i);
  return relative_vel_rtn;
}

void RelativeInformation::ResizeLists() {
  size_t size = dynamics_database_.size();
  rel_pos_list_i_m_.assign(size, vector<libra::Vector<3>>(size, libra::Vector<3>(0)));
  rel_vel_list_i_m_s_.assign(size, vector<libra::Vector<3>>(size, libra::Vector<3>(0)));
  rel_distance_list_m_.assign(size, vector<double>(size, 0.0));
  rel_pos_list_rtn_m_.assign(size, vector<libra::Vector<3>>(size, libra::Vector<3>(0)));
  rel_vel_list_rtn_m_s_.assign(size, vector<libra::Vector<3>>(size, libra::Vector<3>(0)));
  rel_att_quaternion_list_.assign(size, vector<libra::Quaternion>(size, libra::Quaternion(0, 0, 0, 1)));
}
