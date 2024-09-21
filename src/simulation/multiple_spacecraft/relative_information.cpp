/**
 * @file relative_information.cpp
 * @brief Base class to manage relative information between spacecraft
 */

#include "relative_information.hpp"

RelativeInformation::RelativeInformation() {}

RelativeInformation::~RelativeInformation() {}

void RelativeInformation::Update() {
  for (size_t target_spacecraft_id = 0; target_spacecraft_id < dynamics_database_.size(); target_spacecraft_id++) {
    for (size_t reference_spacecraft_id = 0; reference_spacecraft_id < dynamics_database_.size(); reference_spacecraft_id++) {
      // Position
      math::Vector<3> target_sat_pos_i = dynamics_database_.at(target_spacecraft_id)->GetOrbit().GetPosition_i_m();
      math::Vector<3> reference_sat_pos_i = dynamics_database_.at(reference_spacecraft_id)->GetOrbit().GetPosition_i_m();
      relative_position_list_i_m_[target_spacecraft_id][reference_spacecraft_id] = target_sat_pos_i - reference_sat_pos_i;
      relative_position_list_rtn_m_[target_spacecraft_id][reference_spacecraft_id] =
          CalcRelativePosition_rtn_m(target_spacecraft_id, reference_spacecraft_id);

      // Distance
      relative_distance_list_m_[target_spacecraft_id][reference_spacecraft_id] =
          relative_position_list_i_m_[target_spacecraft_id][reference_spacecraft_id].CalcNorm();

      // Velocity
      math::Vector<3> target_sat_vel_i = dynamics_database_.at(target_spacecraft_id)->GetOrbit().GetVelocity_i_m_s();
      math::Vector<3> reference_sat_vel_i = dynamics_database_.at(reference_spacecraft_id)->GetOrbit().GetVelocity_i_m_s();
      relative_velocity_list_i_m_s_[target_spacecraft_id][reference_spacecraft_id] = target_sat_vel_i - reference_sat_vel_i;
      relative_velocity_list_rtn_m_s_[target_spacecraft_id][reference_spacecraft_id] =
          CalcRelativeVelocity_rtn_m_s(target_spacecraft_id, reference_spacecraft_id);

      // Attitude Quaternion
      relative_attitude_quaternion_list_[target_spacecraft_id][reference_spacecraft_id] =
          CalcRelativeAttitudeQuaternion(target_spacecraft_id, reference_spacecraft_id);
    }
  }
}

void RelativeInformation::RegisterDynamicsInfo(const size_t spacecraft_id, const Dynamics* dynamics) {
  dynamics_database_.emplace(spacecraft_id, dynamics);
  ResizeLists();
}

void RelativeInformation::RemoveDynamicsInfo(const size_t spacecraft_id) {
  dynamics_database_.erase(spacecraft_id);
  ResizeLists();
}

std::string RelativeInformation::GetLogHeader() const {
  std::string str_tmp = "";
  for (size_t target_spacecraft_id = 0; target_spacecraft_id < dynamics_database_.size(); target_spacecraft_id++) {
    for (size_t reference_spacecraft_id = 0; reference_spacecraft_id < target_spacecraft_id; reference_spacecraft_id++) {
      str_tmp += WriteVector(
          "satellite" + std::to_string(target_spacecraft_id) + "_position_from_satellite" + std::to_string(reference_spacecraft_id), "i", "m", 3);
    }
  }

  for (size_t target_spacecraft_id = 0; target_spacecraft_id < dynamics_database_.size(); target_spacecraft_id++) {
    for (size_t reference_spacecraft_id = 0; reference_spacecraft_id < target_spacecraft_id; reference_spacecraft_id++) {
      str_tmp += WriteVector(
          "satellite" + std::to_string(target_spacecraft_id) + "_velocity_from_satellite" + std::to_string(reference_spacecraft_id), "i", "m/s", 3);
    }
  }

  for (size_t target_spacecraft_id = 0; target_spacecraft_id < dynamics_database_.size(); target_spacecraft_id++) {
    for (size_t reference_spacecraft_id = 0; reference_spacecraft_id < target_spacecraft_id; reference_spacecraft_id++) {
      str_tmp += WriteVector(
          "satellite" + std::to_string(target_spacecraft_id) + "_position_from_satellite" + std::to_string(reference_spacecraft_id), "rtn", "m", 3);
    }
  }

  for (size_t target_spacecraft_id = 0; target_spacecraft_id < dynamics_database_.size(); target_spacecraft_id++) {
    for (size_t reference_spacecraft_id = 0; reference_spacecraft_id < target_spacecraft_id; reference_spacecraft_id++) {
      str_tmp += WriteVector(
          "satellite" + std::to_string(target_spacecraft_id) + "_velocity_from_satellite" + std::to_string(reference_spacecraft_id), "rtn", "m/s", 3);
    }
  }

  return str_tmp;
}

std::string RelativeInformation::GetLogValue() const {
  std::string str_tmp = "";
  for (size_t target_spacecraft_id = 0; target_spacecraft_id < dynamics_database_.size(); target_spacecraft_id++) {
    for (size_t reference_spacecraft_id = 0; reference_spacecraft_id < target_spacecraft_id; reference_spacecraft_id++) {
      str_tmp += WriteVector(GetRelativePosition_i_m(target_spacecraft_id, reference_spacecraft_id));
    }
  }

  for (size_t target_spacecraft_id = 0; target_spacecraft_id < dynamics_database_.size(); target_spacecraft_id++) {
    for (size_t reference_spacecraft_id = 0; reference_spacecraft_id < target_spacecraft_id; reference_spacecraft_id++) {
      str_tmp += WriteVector(GetRelativeVelocity_i_m_s(target_spacecraft_id, reference_spacecraft_id));
    }
  }

  for (size_t target_spacecraft_id = 0; target_spacecraft_id < dynamics_database_.size(); target_spacecraft_id++) {
    for (size_t reference_spacecraft_id = 0; reference_spacecraft_id < target_spacecraft_id; reference_spacecraft_id++) {
      str_tmp += WriteVector(GetRelativePosition_rtn_m(target_spacecraft_id, reference_spacecraft_id));
    }
  }

  for (size_t target_spacecraft_id = 0; target_spacecraft_id < dynamics_database_.size(); target_spacecraft_id++) {
    for (size_t reference_spacecraft_id = 0; reference_spacecraft_id < target_spacecraft_id; reference_spacecraft_id++) {
      str_tmp += WriteVector(GetRelativeVelocity_rtn_m_s(target_spacecraft_id, reference_spacecraft_id));
    }
  }

  return str_tmp;
}

void RelativeInformation::LogSetup(Logger& logger) { logger.AddLogList(this); }

math::Quaternion RelativeInformation::CalcRelativeAttitudeQuaternion(const size_t target_spacecraft_id, const size_t reference_spacecraft_id) {
  // Observer SC Body frame(obs_sat) -> ECI frame(i)
  math::Quaternion q_reference_i2b = dynamics_database_.at(reference_spacecraft_id)->GetAttitude().GetQuaternion_i2b();
  math::Quaternion q_reference_b2i = q_reference_i2b.Conjugate();

  // ECI frame(i) -> Target SC body frame(main_sat)
  math::Quaternion q_target_i2b = dynamics_database_.at(target_spacecraft_id)->GetAttitude().GetQuaternion_i2b();

  return q_target_i2b * q_reference_b2i;
}

math::Vector<3> RelativeInformation::CalcRelativePosition_rtn_m(const size_t target_spacecraft_id, const size_t reference_spacecraft_id) {
  math::Vector<3> target_sat_pos_i = dynamics_database_.at(target_spacecraft_id)->GetOrbit().GetPosition_i_m();
  math::Vector<3> reference_sat_pos_i = dynamics_database_.at(reference_spacecraft_id)->GetOrbit().GetPosition_i_m();
  math::Vector<3> relative_pos_i = target_sat_pos_i - reference_sat_pos_i;

  // RTN frame for the reference satellite
  math::Quaternion q_i2rtn = dynamics_database_.at(reference_spacecraft_id)->GetOrbit().CalcQuaternion_i2lvlh();

  math::Vector<3> relative_pos_rtn = q_i2rtn.FrameConversion(relative_pos_i);
  return relative_pos_rtn;
}

math::Vector<3> RelativeInformation::CalcRelativeVelocity_rtn_m_s(const size_t target_spacecraft_id, const size_t reference_spacecraft_id) {
  math::Vector<3> target_sat_pos_i = dynamics_database_.at(target_spacecraft_id)->GetOrbit().GetPosition_i_m();
  math::Vector<3> reference_sat_pos_i = dynamics_database_.at(reference_spacecraft_id)->GetOrbit().GetPosition_i_m();
  math::Vector<3> relative_pos_i = target_sat_pos_i - reference_sat_pos_i;

  // RTN frame for the reference satellite
  math::Quaternion q_i2rtn = dynamics_database_.at(reference_spacecraft_id)->GetOrbit().CalcQuaternion_i2lvlh();

  // Rotation vector of RTN frame
  math::Vector<3> reference_sat_vel_i = dynamics_database_.at(reference_spacecraft_id)->GetOrbit().GetVelocity_i_m_s();
  math::Vector<3> target_sat_vel_i = dynamics_database_.at(target_spacecraft_id)->GetOrbit().GetVelocity_i_m_s();
  math::Vector<3> rot_vec_rtn_i = cross(reference_sat_pos_i, reference_sat_vel_i);
  double r2_ref = reference_sat_pos_i.CalcNorm() * reference_sat_pos_i.CalcNorm();
  rot_vec_rtn_i /= r2_ref;
  math::Vector<3> relative_vel_i = target_sat_vel_i - reference_sat_vel_i - cross(rot_vec_rtn_i, relative_pos_i);

  math::Vector<3> relative_vel_rtn = q_i2rtn.FrameConversion(relative_vel_i);
  return relative_vel_rtn;
}

void RelativeInformation::ResizeLists() {
  size_t size = dynamics_database_.size();
  relative_position_list_i_m_.assign(size, std::vector<math::Vector<3>>(size, math::Vector<3>(0)));
  relative_velocity_list_i_m_s_.assign(size, std::vector<math::Vector<3>>(size, math::Vector<3>(0)));
  relative_distance_list_m_.assign(size, std::vector<double>(size, 0.0));
  relative_position_list_rtn_m_.assign(size, std::vector<math::Vector<3>>(size, math::Vector<3>(0)));
  relative_velocity_list_rtn_m_s_.assign(size, std::vector<math::Vector<3>>(size, math::Vector<3>(0)));
  relative_attitude_quaternion_list_.assign(size, std::vector<math::Quaternion>(size, math::Quaternion(0, 0, 0, 1)));
}
