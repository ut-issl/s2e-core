#pragma once
#include "../math/Vector.hpp"

class OrbitalElements {
 public:
  OrbitalElements();
  // initialize with OE
  OrbitalElements(const double epoch_jday, const double semi_major_axis_m, const double eccentricity, const double inclination_rad,
                  const double raan_rad, const double arg_perigee_rad);
  // initialize with position and velocity
  OrbitalElements(const double mu_m3_s2, const double time_jday, const libra::Vector<3> position_i_m, const libra::Vector<3> velocity_i_m_s);
  ~OrbitalElements();

  // Getter
  inline double GetSemiMajor() const { return semi_major_axis_m_; }
  inline double GetEccentricity() const { return eccentricity_; }
  inline double GetInclination() const { return inclination_rad_; }
  inline double GetRaan() const { return raan_rad_; }
  inline double GetArgPerigee() const { return arg_perigee_rad_; }
  inline double GetEpoch() const { return epoch_jday_; }

 private:
  // Common Variables
  // Shape
  double semi_major_axis_m_;
  double eccentricity_;
  // Corrdinate
  double inclination_rad_;
  double raan_rad_;         //!< Right Ascension of the Ascending Node [rad]
  double arg_perigee_rad_;  //!< Argument of Perigee [rad]
  // Time, phase
  double epoch_jday_;  //!< epoch (time at the perigee) [julian day]

  // Functions
  // Calculation of Orbital Elements from Position and Velocity
  void CalcOeFromPosVel(const double mu_m3_s2, const double time_jday, const libra::Vector<3> r_i_m, const libra::Vector<3> v_i_m_s);
};
