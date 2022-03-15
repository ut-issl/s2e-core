#pragma once
#include <string>

#include "../Dynamics/Dynamics.h"
#include "../Interface/LogOutput/ILoggable.h"
#include "../Interface/LogOutput/Logger.h"

class RelativeInformation : public ILoggable {
 public:
  RelativeInformation();
  ~RelativeInformation();
  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;
  void LogSetup(Logger& logger);
  void RegisterDynamicsInfo(const int sat_id, const Dynamics* dynamics);
  void RemoveDynamicsInfo(const int sat_id);

  // Getter

  // Transformation Quaternion: Calculate a quaternion represents the coordinate
  // transformation from the body frame of reference satellite to the body frame
  // of target satellite target_sat_id: satellite ID in relative motion
  // reference_sat_id: satellite ID to observe relative motion
  const libra::Quaternion CalcRelativeAttitudeQuaternion(
      const int target_sat_id, const int reference_sat_id) const;

  const libra::Vector<3> GetRelativePosition_i(
      const int target_sat_id, const int reference_sat_id) const;
  const libra::Vector<3> GetRelativeVelocity_i(
      const int target_sat_id, const int reference_sat_id) const;

  inline const Dynamics* GetReferenceSatDynamics(
      const int reference_sat_id) const {
    return dynamics_database_.at(reference_sat_id);
  };

 private:
  std::map<const int, const Dynamics*> dynamics_database_;
};
