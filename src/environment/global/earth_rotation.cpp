/**
 * @file earth_rotation.cpp
 * @brief Class to calculate the earth rotation
 * @note Refs: 福島,"天体の回転運動理論入門講義ノート", 2007 (in Japanese),
 *             長沢,"天体の位置計算(増補版)", 2001 (in Japanese),
 *             IERS Conventions 2003
 */

#include "earth_rotation.hpp"

#include <iostream>
#include <sstream>

#include "math_physics/math/constants.hpp"
#include "math_physics/orbit/sgp4/sgp4ext.h"   // for jday()
#include "math_physics/orbit/sgp4/sgp4unit.h"  // for gstime()

namespace s2e::environment {

// Default constructor
EarthRotation::EarthRotation(const EarthRotationMode rotation_mode) : rotation_mode_(rotation_mode) {
  dcm_j2000_to_ecef_ = math::MakeIdentityMatrix<3>();
  dcm_teme_to_ecef_ = dcm_j2000_to_ecef_;
  InitializeParameters();
}

// Initialize the class EarthRotation instance as Earth
void EarthRotation::InitializeParameters() {
  if (rotation_mode_ == EarthRotationMode::kSimple) {
    // For Simple mode, we don't need initialization of the coefficients
  } else if (rotation_mode_ == EarthRotationMode::kFull) {
    // For Full mode, initialize the coefficients
    // The hard coded values are consistent with the reference document. The unit deg and rad are mixed.

    // Coefficients to compute mean obliquity of the ecliptic
    // The actual unit of the coefficients are [rad/century^i], where i is the index of the array
    c_epsilon_rad_[0] = 23.4392911 * math::deg_to_rad;      // [rad]
    c_epsilon_rad_[1] = -46.8150000 * math::arcsec_to_rad;  // [rad/century]
    c_epsilon_rad_[2] = -5.9000e-4 * math::arcsec_to_rad;   // [rad/century^2]
    c_epsilon_rad_[3] = 1.8130e-3 * math::arcsec_to_rad;    // [rad/century^3]

    // Coefficients to compute Delaunay angles
    // The actual unit of the coefficients are [rad/century^i], where i is the index of the array
    c_lm_rad_[0] = 134.96340251 * math::deg_to_rad;            // [rad]
    c_lm_rad_[1] = 1717915923.21780000 * math::arcsec_to_rad;  // [rad/century]
    c_lm_rad_[2] = 31.87920000 * math::arcsec_to_rad;          // [rad/century^2]
    c_lm_rad_[3] = 0.05163500 * math::arcsec_to_rad;           // [rad/century^3]
    c_lm_rad_[4] = -0.00024470 * math::arcsec_to_rad;          // [rad/century^4]

    c_ls_rad_[0] = 357.52910918 * math::deg_to_rad;           // [rad]
    c_ls_rad_[1] = 129596581.04810000 * math::arcsec_to_rad;  // [rad/century]
    c_ls_rad_[2] = -0.55320000 * math::arcsec_to_rad;         // [rad/century^2]
    c_ls_rad_[3] = 0.00013600 * math::arcsec_to_rad;          // [rad/century^3]
    c_ls_rad_[4] = -0.00001149 * math::arcsec_to_rad;         // [rad/century^4]

    c_f_rad_[0] = 93.27209062 * math::deg_to_rad;             // [rad]
    c_f_rad_[1] = 1739527262.84780000 * math::arcsec_to_rad;  // [rad/century]
    c_f_rad_[2] = -12.75120000 * math::arcsec_to_rad;         // [rad/century^2]
    c_f_rad_[3] = -0.00103700 * math::arcsec_to_rad;          // [rad/century^3]
    c_f_rad_[4] = 0.00000417 * math::arcsec_to_rad;           // [rad/century^4]

    c_d_rad_[0] = 297.85019547 * math::deg_to_rad;            // [rad]
    c_d_rad_[1] = 1602961601.20900000 * math::arcsec_to_rad;  // [rad/century]
    c_d_rad_[2] = -6.37060000 * math::arcsec_to_rad;          // [rad/century^2]
    c_d_rad_[3] = 0.00659300 * math::arcsec_to_rad;           // [rad/century^3]
    c_d_rad_[4] = -0.00003169 * math::arcsec_to_rad;          // [rad/century^4]

    c_o_rad_[0] = 125.04455501 * math::deg_to_rad;          // [rad]
    c_o_rad_[1] = -6962890.54310000 * math::arcsec_to_rad;  // [rad/century]
    c_o_rad_[2] = 7.47220000 * math::arcsec_to_rad;         // [rad/century^2]
    c_o_rad_[3] = 0.00770200 * math::arcsec_to_rad;         // [rad/century^3]
    c_o_rad_[4] = -0.00005939 * math::arcsec_to_rad;        // [rad/century^4]

    // Coefficients to compute nutation angles
    c_d_epsilon_rad_[0] = 9.2050 * math::arcsec_to_rad;   // [rad]
    c_d_epsilon_rad_[1] = 0.5730 * math::arcsec_to_rad;   // [rad]
    c_d_epsilon_rad_[2] = -0.0900 * math::arcsec_to_rad;  // [rad]
    c_d_epsilon_rad_[3] = 0.0980 * math::arcsec_to_rad;   // [rad]
    c_d_epsilon_rad_[4] = 0.0070 * math::arcsec_to_rad;   // [rad]
    c_d_epsilon_rad_[5] = -0.0010 * math::arcsec_to_rad;  // [rad]
    c_d_epsilon_rad_[6] = 0.0220 * math::arcsec_to_rad;   // [rad]
    c_d_epsilon_rad_[7] = 0.0130 * math::arcsec_to_rad;   // [rad]
    c_d_epsilon_rad_[8] = -0.0100 * math::arcsec_to_rad;  // [rad]

    c_d_psi_rad_[0] = -17.2060 * math::arcsec_to_rad;  // [rad]
    c_d_psi_rad_[1] = -1.3170 * math::arcsec_to_rad;   // [rad]
    c_d_psi_rad_[2] = 0.2070 * math::arcsec_to_rad;    // [rad]
    c_d_psi_rad_[3] = -0.2280 * math::arcsec_to_rad;   // [rad]
    c_d_psi_rad_[4] = 0.1480 * math::arcsec_to_rad;    // [rad]
    c_d_psi_rad_[5] = 0.0710 * math::arcsec_to_rad;    // [rad]
    c_d_psi_rad_[6] = -0.0520 * math::arcsec_to_rad;   // [rad]
    c_d_psi_rad_[7] = -0.0300 * math::arcsec_to_rad;   // [rad]
    c_d_psi_rad_[8] = 0.0220 * math::arcsec_to_rad;    // [rad]

    // Coefficients to compute precession angle
    // The actual unit of the coefficients are [rad/century^i], where i is the index of the array
    c_zeta_rad_[0] = 2306.218100 * math::arcsec_to_rad;  // [rad/century]
    c_zeta_rad_[1] = 0.301880 * math::arcsec_to_rad;     // [rad/century^2]
    c_zeta_rad_[2] = 0.017998 * math::arcsec_to_rad;     // [rad/century^3]

    c_theta_rad_[0] = 2004.310900 * math::arcsec_to_rad;  // [rad/century]
    c_theta_rad_[1] = -0.426650 * math::arcsec_to_rad;    // [rad/century^2]
    c_theta_rad_[2] = -0.041833 * math::arcsec_to_rad;    // [rad/century^3]

    c_z_rad_[0] = 2306.218100 * math::arcsec_to_rad;  // [rad/century]
    c_z_rad_[1] = 1.094680 * math::arcsec_to_rad;     // [rad/century^2]
    c_z_rad_[2] = 0.018203 * math::arcsec_to_rad;     // [rad/century^3]
  } else {
    // If the rotation mode is neither Simple nor Full, disable the rotation calculation and make the DCM a unit matrix
    dcm_j2000_to_ecef_ = math::MakeIdentityMatrix<3>();
  }
}

void EarthRotation::Update(const double julian_date) {
  double gmst_rad = gstime(julian_date);  // It is a bit different with 長沢(Nagasawa)'s algorithm. TODO: Check the correctness

  if (rotation_mode_ == EarthRotationMode::kFull) {
    // Compute Julian date for terrestrial time
    double terrestrial_time_julian_day =
        julian_date + kDtUt1Utc_ * kSec2Day_;  // TODO: Check the correctness. Problem is that S2E doesn't have Gregorian calendar.

    // Compute nth power of julian century for terrestrial time.
    // The actual unit of tTT_century is [century^(i+1)], i is the index of the array
    double terrestrial_time_julian_century[4];
    terrestrial_time_julian_century[0] = (terrestrial_time_julian_day - kJulianDateJ2000_) / kDayJulianCentury_;
    for (int i = 0; i < 3; i++) {
      terrestrial_time_julian_century[i + 1] = terrestrial_time_julian_century[i] * terrestrial_time_julian_century[0];
    }

    math::Matrix<3, 3> dcm_precession;
    math::Matrix<3, 3> dcm_nutation;
    math::Matrix<3, 3> dcm_rotation;
    math::Matrix<3, 3> dcm_polar_motion;
    // Nutation + Precession
    dcm_precession = Precession(terrestrial_time_julian_century);
    dcm_nutation = Nutation(terrestrial_time_julian_century);  // epsilon_rad_, d_epsilon_rad_, d_psi_rad_ are updated in this procedure

    // Axial Rotation
    double equinox_rad = d_psi_rad_ * cos(epsilon_rad_ + d_epsilon_rad_);  // Equation of equinoxes [rad]
    double gast_rad = gmst_rad + equinox_rad;                              // Greenwich 'Apparent' Sidereal Time [rad]
    dcm_rotation = AxialRotation(gast_rad);
    // Polar motion (is not considered so far, even without polar motion, the result agrees well with the matlab reference)
    double x_p = 0.0;
    double y_p = 0.0;
    dcm_polar_motion = PolarMotion(x_p, y_p);

    // Total orientation
    dcm_j2000_to_ecef_ = dcm_polar_motion * dcm_rotation * dcm_nutation * dcm_precession;
  } else if (rotation_mode_ == EarthRotationMode::kSimple) {
    // In this case, only Axial Rotation is executed, with its argument replaced from G'A'ST to G'M'ST
    // FIXME: Not suitable when the center body is not the earth
    dcm_j2000_to_ecef_ = AxialRotation(gmst_rad);
  } else {
    // Leave the DCM as unit Matrix(diag{1,1,1})
    return;
  }
}

math::Matrix<3, 3> EarthRotation::AxialRotation(const double gast_rad) { return math::MakeRotationMatrixZ(gast_rad); }

math::Matrix<3, 3> EarthRotation::Nutation(const double (&t_tt_century)[4]) {
  // Mean obliquity of the ecliptic
  epsilon_rad_ = c_epsilon_rad_[0];
  for (int i = 0; i < 3; i++) {
    epsilon_rad_ += c_epsilon_rad_[i + 1] * t_tt_century[i];
  }

  // Compute five Delaunay angles(l=lm, l'=ls, F, D, Ω=O)
  // Mean anomaly of the moon
  double lm_rad = c_lm_rad_[0];
  for (int i = 0; i < 4; i++) {
    lm_rad += c_lm_rad_[i + 1] * t_tt_century[i];
  }
  // Mean anomaly of the sun
  double ls_rad = c_ls_rad_[0];
  for (int i = 0; i < 4; i++) {
    ls_rad += c_ls_rad_[i + 1] * t_tt_century[i];
  }
  // Mean longitude of the moon - mean longitude of ascending node of the moon
  double f_rad = c_f_rad_[0];
  for (int i = 0; i < 4; i++) {
    f_rad += c_f_rad_[i + 1] * t_tt_century[i];
  }
  // Mean elongation of the moon from the sun
  double d_rad = c_d_rad_[0];
  for (int i = 0; i < 4; i++) {
    d_rad += c_d_rad_[i + 1] * t_tt_century[i];
  }
  // Mean longitude of ascending node of the moon
  double o_rad = c_o_rad_[0];
  for (int i = 0; i < 4; i++) {
    o_rad += c_o_rad_[i + 1] * t_tt_century[i];
  }

  // Additional angles
  double l_rad = f_rad + o_rad;   // F + Ω
  double ld_rad = l_rad - d_rad;  // F + Ω - D

  // Compute luni-solar nutation
  // Nutation in obliquity
  d_psi_rad_ = c_d_psi_rad_[0] * sin(o_rad) + c_d_psi_rad_[1] * sin(2 * ld_rad) + c_d_psi_rad_[2] * sin(2 * o_rad) +
               c_d_psi_rad_[3] * sin(2 * l_rad) + c_d_psi_rad_[4] * sin(ls_rad);
  d_psi_rad_ = d_psi_rad_ + c_d_psi_rad_[5] * sin(lm_rad) + c_d_psi_rad_[6] * sin(2 * ld_rad + ls_rad) + c_d_psi_rad_[7] * sin(2 * l_rad + lm_rad) +
               c_d_psi_rad_[8] * sin(2 * ld_rad - ls_rad);

  // Nutation in longitude
  d_epsilon_rad_ = c_d_epsilon_rad_[0] * cos(o_rad) + c_d_epsilon_rad_[1] * cos(2 * ld_rad) + c_d_epsilon_rad_[2] * cos(2 * o_rad) +
                   c_d_epsilon_rad_[3] * cos(2 * l_rad) + c_d_epsilon_rad_[4] * cos(ls_rad);
  d_epsilon_rad_ = d_epsilon_rad_ + c_d_epsilon_rad_[5] * cos(lm_rad) + c_d_epsilon_rad_[6] * cos(2 * ld_rad + ls_rad) +
                   c_d_epsilon_rad_[7] * cos(2 * l_rad + lm_rad) + c_d_epsilon_rad_[8] * cos(2 * ld_rad - ls_rad);

  double epsi_mod_rad = epsilon_rad_ + d_epsilon_rad_;
  math::Matrix<3, 3> x_epsi_1st = math::MakeRotationMatrixX(epsilon_rad_);
  math::Matrix<3, 3> z_d_psi = math::MakeRotationMatrixZ(-d_psi_rad_);
  math::Matrix<3, 3> x_epsi_2nd = math::MakeRotationMatrixX(-epsi_mod_rad);

  math::Matrix<3, 3> dcm_nutation;
  dcm_nutation = x_epsi_2nd * z_d_psi * x_epsi_1st;

  return dcm_nutation;
}

math::Matrix<3, 3> EarthRotation::Precession(const double (&t_tt_century)[4]) {
  // Compute precession angles(zeta, theta, z)
  double zeta_rad = 0.0;
  for (int i = 0; i < 3; i++) {
    zeta_rad += c_zeta_rad_[i] * t_tt_century[i];
  }
  double theta_rad = 0.0;
  for (int i = 0; i < 3; i++) {
    theta_rad += c_theta_rad_[i] * t_tt_century[i];
  }
  double z_rad = 0.0;
  for (int i = 0; i < 3; i++) {
    z_rad += c_z_rad_[i] * t_tt_century[i];
  }

  // Develop transformation matrix
  math::Matrix<3, 3> z_zeta = math::MakeRotationMatrixZ(-zeta_rad);
  math::Matrix<3, 3> y_theta = math::MakeRotationMatrixY(theta_rad);
  math::Matrix<3, 3> z_z = math::MakeRotationMatrixZ(-z_rad);

  math::Matrix<3, 3> dcm_precession;
  dcm_precession = z_z * y_theta * z_zeta;

  return dcm_precession;
}

math::Matrix<3, 3> EarthRotation::PolarMotion(const double x_p, const double y_p) {
  math::Matrix<3, 3> dcm_polar_motion;

  dcm_polar_motion[0][0] = 1.0;
  dcm_polar_motion[0][1] = 0.0;
  dcm_polar_motion[0][2] = -x_p;
  dcm_polar_motion[1][0] = 0.0;
  dcm_polar_motion[1][1] = 1.0;
  dcm_polar_motion[1][2] = -y_p;
  dcm_polar_motion[2][0] = x_p;
  dcm_polar_motion[2][1] = y_p;
  dcm_polar_motion[2][2] = 1.0;

  return dcm_polar_motion;
}

EarthRotationMode ConvertEarthRotationMode(const std::string mode) {
  EarthRotationMode rotation_mode;
  if (mode == "IDLE") {
    rotation_mode = EarthRotationMode::kIdle;
  } else if (mode == "SIMPLE") {
    rotation_mode = EarthRotationMode::kSimple;
  } else if (mode == "FULL") {
    rotation_mode = EarthRotationMode::kFull;
  } else  // if rotation_mode is neither Idle, Simple, nor Full, set rotation_mode to Idle
  {
    rotation_mode = EarthRotationMode::kIdle;
  }

  return rotation_mode;
}

} // namespace s2e::environment
