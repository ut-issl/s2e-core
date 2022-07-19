#pragma once
#include <string>

#include "../Dynamics/Dynamics.h"
#include "../Interface/LogOutput/ILoggable.h"
#include "../Interface/LogOutput/Logger.h"

class RelativeInformation : public ILoggable {
 public:
  RelativeInformation();
  ~RelativeInformation();

  void Update();
  void RegisterDynamicsInfo(const int sat_id, const Dynamics* dynamics);
  void RemoveDynamicsInfo(const int sat_id);

  // ILoggable
  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;
  void LogSetup(Logger& logger);

  // Getter
  inline libra::Quaternion GetRelativeAttitudeQuaternion(const int target_sat_id, const int reference_sat_id) const {
    return rel_att_quaternion_list_[target_sat_id][reference_sat_id];
  }
  inline libra::Vector<3> GetRelativePosition_i_m(const int target_sat_id, const int reference_sat_id) const {
    return rel_pos_list_i_m_[target_sat_id][reference_sat_id];
  }
  inline libra::Vector<3> GetRelativeVelocity_i_m_s(const int target_sat_id, const int reference_sat_id) const {
    return rel_vel_list_i_m_s_[target_sat_id][reference_sat_id];
  }
  inline double GetRelativeDistance_m(const int target_sat_id, const int reference_sat_id) const {
    return rel_distance_list_m_[target_sat_id][reference_sat_id];
  };
  inline libra::Vector<3> GetRelativePosition_rtn_m(const int target_sat_id, const int reference_sat_id) const {
    return rel_pos_list_rtn_m_[target_sat_id][reference_sat_id];
  }

  inline const Dynamics* GetReferenceSatDynamics(const int reference_sat_id) const { return dynamics_database_.at(reference_sat_id); };

 private:
  std::map<const int, const Dynamics*> dynamics_database_;

  std::vector<std::vector<libra::Vector<3>>> rel_pos_list_i_m_;
  std::vector<std::vector<libra::Vector<3>>> rel_vel_list_i_m_s_;
  std::vector<std::vector<libra::Vector<3>>> rel_pos_list_rtn_m_;
  std::vector<std::vector<double>> rel_distance_list_m_;
  std::vector<std::vector<libra::Quaternion>> rel_att_quaternion_list_;

  libra::Quaternion CalcRelativeAttitudeQuaternion(const int target_sat_id, const int reference_sat_id);
  libra::Vector<3> CalcRelativePosition_rtn_m(const int target_sat_id, const int reference_sat_id);
  void ResizeLists();
};
