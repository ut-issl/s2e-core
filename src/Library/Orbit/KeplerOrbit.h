#pragma once
#include "./OrbitalElements.h"
#include "../math/Matrix.hpp"
#include "../math/Vector.hpp"

class KeplerOrbit
{
public:
  KeplerOrbit();
  // Initialize with orbital elements
  KeplerOrbit(
    const double mu_m3_s2,
    const double current_jd,
    const OrbitalElements oe
  );
  ~KeplerOrbit();

  void CalcPosVel(double time_jday); // Calculation of Position and Velocity from Orbital Elements

  inline const libra::Vector<3> GetPosition_i_m() const {return position_i_m_;}
  inline const libra::Vector<3> GetVelocity_i_m_s() const {return velocity_i_m_s_;}

protected:
  libra::Vector<3> position_i_m_;
  libra::Vector<3> velocity_i_m_s_;

private:
  double mu_m3_s2_;
  OrbitalElements oe_; 
  double mean_motion_rad_s_;
  libra::Matrix<3, 3> dcm_inplane_to_eci_;

  // Calculation of constants for kepler motion calculation
  void CalcConstKeplerMotion();
  // Solve Kepler Equation
  double SolveKeplerFirstOrder(
    const double eccentricity, 
    const double mean_anomaly_rad, 
    const double angle_limit_rad, 
    const int iteration_limit
  );
};
