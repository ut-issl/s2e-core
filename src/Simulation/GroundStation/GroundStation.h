#pragma once

#include <Environment/Global/CelestialRotation.h>
#include <Interface/InitInput/Initialize.h>

#include <Library/Geodesy/GeodeticPosition.hpp>
#include <Library/math/Vector.hpp>

#include "../SimulationConfig.h"

class GroundStation {
 public:
  int gs_id_;               // GroundStationのID
  double elevation_angle_;  //[deg]

  GroundStation(SimulationConfig* config, int gs_id_);
  virtual ~GroundStation();

  // ちょっとコピー周りの対応がめんどいのでとりあえずコピー禁止しとく
  // GroundStation(const GroundStation &) = delete;
  // GroundStation& operator= (const GroundStation &) = delete;

  virtual void Initialize(int gs_id, SimulationConfig* config);

  virtual void LogSetup(Logger& logger);

  virtual void Update(const CelestialRotation& celes_rotation);

  GeodeticPosition GetGSPosition_geo() const { return gs_position_geo_; }
  Vector<3> GetGSPosition_ecef() const { return gs_position_ecef_; }
  Vector<3> GetGSPosition_i() const { return gs_position_i_; }

 protected:
  GeodeticPosition gs_position_geo_;  //! Ground Station Position in the geodetic frame
  Vector<3> gs_position_ecef_;        //! Ground Station Position in the ECEF frame [m]
  Vector<3> gs_position_i_;           //! Ground Station Position in the inertial frame [m]
};
