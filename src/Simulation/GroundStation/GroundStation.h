#pragma once

#include <Environment/Global/CelestialRotation.h>

#include <Library/Geodesy/GeodeticPosition.hpp>
#include <Library/math/Vector.hpp>

#include "../SimulationConfig.h"

class GroundStation {
 public:
  GroundStation(SimulationConfig* config, int gs_id_);
  virtual ~GroundStation();

  // ちょっとコピー周りの対応がめんどいのでとりあえずコピー禁止しとく
  // GroundStation(const GroundStation &) = delete;
  // GroundStation& operator= (const GroundStation &) = delete;

  // Virtual functions
  virtual void Initialize(int gs_id, SimulationConfig* config);
  virtual void LogSetup(Logger& logger);
  virtual void Update(const CelestialRotation& celes_rotation);
  // TODO: Do we actually need the update function to calculate the GS position in ECI frame?

  // Getters
  int GetGsId() const { return gs_id_; }
  GeodeticPosition GetGSPosition_geo() const { return gs_position_geo_; }
  Vector<3> GetGSPosition_ecef() const { return gs_position_ecef_; }
  Vector<3> GetGSPosition_i() const { return gs_position_i_; }
  double GetElevationLimitAngle_deg() const { return elevation_limit_angle_deg_; }

 protected:
  int gs_id_;                         //! Ground station ID
  GeodeticPosition gs_position_geo_;  //! Ground Station Position in the geodetic frame
  Vector<3> gs_position_ecef_;        //! Ground Station Position in the ECEF frame [m]
  Vector<3> gs_position_i_;           //! Ground Station Position in the inertial frame [m]
  double elevation_limit_angle_deg_;  //! Minimum elevation angle to work the ground station
};
