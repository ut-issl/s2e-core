#include "RelativeInformation.h"

RelativeInformation::RelativeInformation() {}

RelativeInformation::~RelativeInformation() {}

void RelativeInformation::RegisterDynamicsInfo(const int sat_id, const Dynamics* dynamics) { dynamics_database_.emplace(sat_id, dynamics); }

void RelativeInformation::RemoveDynamicsInfo(const int sat_id) { dynamics_database_.erase(sat_id); }

const libra::Quaternion RelativeInformation::CalcRelativeAttitudeQuaternion(const int target_sat_id, const int reference_sat_id) const {
  // Observer SC Body frame(obs_sat) -> ECI frame(i)
  Quaternion q_reference_i2b = dynamics_database_.at(reference_sat_id)->GetAttitude().GetQuaternion_i2b();
  Quaternion q_reference_b2i = q_reference_i2b.conjugate();

  // ECI frame(i) -> Target SC body frame(main_sat)
  Quaternion q_target_i2b = dynamics_database_.at(target_sat_id)->GetAttitude().GetQuaternion_i2b();

  return q_target_i2b * q_reference_b2i;
}

const libra::Vector<3> RelativeInformation::GetRelativePosition_i(const int target_sat_id, const int reference_sat_id) const {
  libra::Vector<3> target_sat_pos_i = dynamics_database_.at(target_sat_id)->GetOrbit().GetSatPosition_i();
  libra::Vector<3> reference_sat_pos_i = dynamics_database_.at(reference_sat_id)->GetOrbit().GetSatPosition_i();
  return target_sat_pos_i - reference_sat_pos_i;
}

const libra::Vector<3> RelativeInformation::GetRelativeVelocity_i(const int target_sat_id, const int reference_sat_id) const {
  libra::Vector<3> target_sat_vel_i = dynamics_database_.at(target_sat_id)->GetOrbit().GetSatVelocity_i();
  libra::Vector<3> reference_sat_vel_i = dynamics_database_.at(reference_sat_id)->GetOrbit().GetSatVelocity_i();
  return target_sat_vel_i - reference_sat_vel_i;
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
      str_tmp += WriteVector("sat" + std::to_string(target_sat_id) + " velocity from sat" + std::to_string(reference_sat_id), "i", "m", 3);
    }
  }
  return str_tmp;
}

std::string RelativeInformation::GetLogValue() const {
  std::string str_tmp = "";
  for (size_t target_sat_id = 0; target_sat_id < dynamics_database_.size(); target_sat_id++) {
    for (size_t reference_sat_id = 0; reference_sat_id < target_sat_id; reference_sat_id++) {
      str_tmp += WriteVector(GetRelativePosition_i(target_sat_id, reference_sat_id));
    }
  }

  for (size_t target_sat_id = 0; target_sat_id < dynamics_database_.size(); target_sat_id++) {
    for (size_t reference_sat_id = 0; reference_sat_id < target_sat_id; reference_sat_id++) {
      str_tmp += WriteVector(GetRelativeVelocity_i(target_sat_id, reference_sat_id));
    }
  }
  return str_tmp;
}

void RelativeInformation::LogSetup(Logger& logger) { logger.AddLoggable(this); }
