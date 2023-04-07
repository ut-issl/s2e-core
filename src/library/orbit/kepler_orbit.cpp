/**
 * @file kepler_orbit.cpp
 * @brief Class to calculate Kepler orbit calculation
 */
#include "kepler_orbit.hpp"

#include "../math/matrix_vector.hpp"
#include "../math/s2e_math.hpp"

KeplerOrbit::KeplerOrbit() {}
// Initialize with orbital elements
KeplerOrbit::KeplerOrbit(const double gravity_constant_m3_s2, const OrbitalElements oe) : gravity_constant_m3_s2_(gravity_constant_m3_s2), oe_(oe) {
  CalcConstKeplerMotion();
}

KeplerOrbit::~KeplerOrbit() {}

// Private Functions
void KeplerOrbit::CalcConstKeplerMotion() {
  // mean motion
  double a_m3 = pow(oe_.GetSemiMajorAxis_m(), 3.0);
  mean_motion_rad_s_ = sqrt(gravity_constant_m3_s2_ / a_m3);

  // DCM
  libra::Matrix<3, 3> dcm_arg_perigee = libra::MakeRotationMatrixZ(-1.0 * oe_.GetArgPerigee_rad());
  libra::Matrix<3, 3> dcm_inclination = libra::MakeRotationMatrixX(-1.0 * oe_.GetInclination_rad());
  libra::Matrix<3, 3> dcm_raan = libra::MakeRotationMatrixZ(-1.0 * oe_.GetRaan_rad());
  libra::Matrix<3, 3> dcm_inc_arg = dcm_inclination * dcm_arg_perigee;
  dcm_inplane_to_i_ = dcm_raan * dcm_inc_arg;
}

void KeplerOrbit::CalcOrbit(double time_jday) {
  // replace to short name variables
  double a_m = oe_.GetSemiMajorAxis_m();
  double e = oe_.GetEccentricity();
  double n_rad_s = mean_motion_rad_s_;
  double dt_s = (time_jday - oe_.GetEpoch_jday()) * (24.0 * 60.0 * 60.0);

  double mean_anomaly_rad = mean_motion_rad_s_ * dt_s;
  double l_rad = libra::WrapTo2Pi(mean_anomaly_rad);

  // Solve Kepler Equation
  double eccentric_anomaly_rad;
  eccentric_anomaly_rad = SolveKeplerNewtonMethod(e, l_rad, 1.0e-5, 10);
  double u_rad = libra::WrapTo2Pi(eccentric_anomaly_rad);

  // Calc position and velocity in the plane
  double cos_u = cos(u_rad);
  double sin_u = sin(u_rad);
  double a_sqrt_e_m = a_m * sqrt(1.0 - e * e);
  double e_cos_u = 1.0 - e * cos_u;

  libra::Vector<3> pos_inplane_m;
  pos_inplane_m[0] = a_m * (cos_u - e);
  pos_inplane_m[1] = a_sqrt_e_m * sin_u;
  pos_inplane_m[2] = 0.0;

  libra::Vector<3> vel_inplane_m_s;
  vel_inplane_m_s[0] = -1.0 * a_m * n_rad_s * sin_u / e_cos_u;
  vel_inplane_m_s[1] = n_rad_s * a_sqrt_e_m * cos_u / e_cos_u;
  vel_inplane_m_s[2] = 0.0;

  // Transform to ECI
  position_i_m_ = dcm_inplane_to_i_ * pos_inplane_m;
  velocity_i_m_s_ = dcm_inplane_to_i_ * vel_inplane_m_s;
}

double KeplerOrbit::SolveKeplerFirstOrder(const double eccentricity, const double mean_anomaly_rad, const double angle_limit_rad,
                                          const int iteration_limit) {
  double u_prev_rad = mean_anomaly_rad;
  double u_rad = 0.0;

  for (int i = 0; i < iteration_limit; i++) {
    u_rad = mean_anomaly_rad + eccentricity * sin(u_prev_rad);

    double diff_abs_rad = std::abs(u_rad - u_prev_rad);
    if (diff_abs_rad < angle_limit_rad) break;

    u_prev_rad = u_rad;
  }

  return u_rad;
}

double KeplerOrbit::SolveKeplerNewtonMethod(const double eccentricity, const double mean_anomaly_rad, const double angle_limit_rad,
                                            const int iteration_limit) {
  double u_prev_rad = mean_anomaly_rad;
  double u_rad = 0.0;

  for (int i = 0; i < iteration_limit; i++) {
    u_rad -= (u_prev_rad - eccentricity * sin(u_prev_rad) - mean_anomaly_rad) / (1.0 - eccentricity * cos(u_prev_rad));

    double diff_abs_rad = std::abs(u_rad - u_prev_rad);
    if (diff_abs_rad < angle_limit_rad) break;

    u_prev_rad = u_rad;
  }
  return u_rad;
}
