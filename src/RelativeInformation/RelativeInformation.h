#pragma once
#include "../Interface/LogOutput/ILoggable.h"
#include "../Interface/LogOutput/Logger.h"
#include "../Dynamics/Dynamics.h"

class RelativeInformation : public ILoggable
{
public:
  RelativeInformation();
  ~RelativeInformation();
  virtual string GetLogHeader() const;
  virtual string GetLogValue() const;
  void LogSetup(Logger& logger);
  void RegisterDynamicsInfo(const int sat_id, const Dynamics* dynamics);

  //Getter

  // Transformation Quaternion: Calculate a quaternion represents the coordinate transformation from the body frame of reference satellite to the body frame of target satellite
  // target_sat_id: satellite ID in relative motion
  // reference_sat_id: satellite ID to observe relative motion
  const libra::Quaternion CalcRelativeAttitudeQuaternion(const int target_sat_id, const int reference_sat_id) const;

  const libra::Vector<3> GetRelativePosition_i(const int target_sat_id, const int reference_sat_id) const;
  const libra::Vector<3> GetRelativeVelocity_i(const int target_sat_id, const int reference_sat_id) const;

private:
  std::map<const int, const Dynamics*> dynamics_database_;
};
