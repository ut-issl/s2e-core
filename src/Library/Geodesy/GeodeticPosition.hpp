#pragma once

#include <Library/math/Vector.hpp>

class GeodeticPosition {
 public:
  GeodeticPosition();
  GeodeticPosition(const double latitude_rad, const double longitude_rad, const double altitude_m);
  void UpdateFromEcef(const libra::Vector<3> position_ecef_m);
  libra::Vector<3> CalcEcefPosition() const;

  // Getter
  inline double GetLat_rad() const { return latitude_rad_; }
  inline double GetLon_rad() const { return longitude_rad_; }
  inline double GetAlt_m() const { return altitude_m_; }

 private:
  double latitude_rad_;   //! South: -π/2 to 0, North: 0 to π/2
  double longitude_rad_;  //! East longitude: 0 to π, West longitude: 2π to π (i.e., defined as 0 to 2π [rad] east of the Greenwich meridian)
  double altitude_m_;     //! Altitude
};
