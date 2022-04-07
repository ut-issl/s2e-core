#include "OrbitalElements.h"

#include "../math/s2e_math.hpp"

OrbitalElements::OrbitalElements() {}

// initialize with OE
OrbitalElements::OrbitalElements(const double epoch_jday, const double semi_major_axis_m, const double eccentricity, const double inclination_rad,
                                 const double raan_rad, const double arg_perigee_rad)
    : semi_major_axis_m_(semi_major_axis_m),
      eccentricity_(eccentricity),
      inclination_rad_(inclination_rad),
      raan_rad_(raan_rad),
      arg_perigee_rad_(arg_perigee_rad),
      epoch_jday_(epoch_jday) {}

// initialize with position and velocity
OrbitalElements::OrbitalElements(const double mu_m3_s2, const double time_jday, const libra::Vector<3> position_i_m,
                                 const libra::Vector<3> velocity_i_m_s) {
  CalcOeFromPosVel(mu_m3_s2, time_jday, position_i_m, velocity_i_m_s);
}

OrbitalElements::~OrbitalElements() {}

// Private Function
void OrbitalElements::CalcOeFromPosVel(const double mu_m3_s2, const double time_jday, const libra::Vector<3> r_i_m, const libra::Vector<3> v_i_m_s) {
  // common variables
  double r_m = norm(r_i_m);
  double r2_m2 = inner_product(r_i_m, r_i_m);
  double v_m_s = norm(v_i_m_s);
  double v2_m2_s2 = inner_product(v_i_m_s, v_i_m_s);
  libra::Vector<3> h;
  h = outer_product(r_i_m, v_i_m_s);
  double h_norm = norm(h);

  // semi major axis
  semi_major_axis_m_ = mu_m3_s2 / (2.0 * mu_m3_s2 / r_m - v2_m2_s2);

  // inclination
  libra::Vector<3> h_direction = h;
  h_direction = normalize(h_direction);
  inclination_rad_ = acos(h_direction[2]);

  // RAAN
  raan_rad_ = asin(h[0] / sqrt(h[0] * h[0] + h[1] * h[1]));  // TODO: 傾斜角0に対応できない

  // position in plain
  double x_p_m = r_i_m[0] * cos(raan_rad_) + r_i_m[1] * sin(raan_rad_);
  double tmp_m = -r_i_m[0] * sin(raan_rad_) + r_i_m[1] * cos(raan_rad_);
  double y_p_m = tmp_m * cos(inclination_rad_) + r_i_m[2] * sin(inclination_rad_);

  // velocity in plain
  double dx_p_m_s = v_i_m_s[0] * cos(raan_rad_) + v_i_m_s[1] * sin(raan_rad_);
  double dtmp_m_s = -v_i_m_s[0] * sin(raan_rad_) + v_i_m_s[1] * cos(raan_rad_);
  double dy_p_m_s = dtmp_m_s * cos(inclination_rad_) + v_i_m_s[2] * sin(inclination_rad_);

  // eccentricity
  double t1 = (h_norm / mu_m3_s2) * dy_p_m_s - x_p_m / r_m;
  double t2 = -(h_norm / mu_m3_s2) * dx_p_m_s - y_p_m / r_m;
  eccentricity_ = sqrt(t1 * t1 + t2 * t2);

  // arg perigee
  arg_perigee_rad_ = atan2(t2, t1);

  // true anomaly f_rad and eccentric anomaly u_rad
  double phi_rad = atan2(y_p_m, x_p_m);
  double f_rad = phi_rad - arg_perigee_rad_;
  f_rad = libra::WrapTo2Pi(f_rad);

  double u_rad = atan2(r_m * sin(f_rad) / sqrt(1.0 - eccentricity_ * eccentricity_), r_m * cos(f_rad) + semi_major_axis_m_ * eccentricity_);
  u_rad = libra::WrapTo2Pi(u_rad);

  // epoch t0
  double n_rad_s = sqrt(mu_m3_s2 / pow(semi_major_axis_m_, 3.0));
  double dt_s = (u_rad - eccentricity_ * sin(u_rad)) / n_rad_s;
  epoch_jday_ = time_jday - dt_s / (24.0 * 60.0 * 60.0);
}
