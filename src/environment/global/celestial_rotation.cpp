/**
 * @file celestial_rotation.cpp
 * @brief Class to calculate the celestial rotation
 * @note Support earth rotation only now (TODO: add other planets)
 *       Refs: 福島,"天体の回転運動理論入門講義ノート", 2007 (in Japanese),
 *             長沢,"天体の位置計算(増補版)", 2001 (in Japanese),
 *             IERS Conventions 2003
 */

#include "celestial_rotation.hpp"

#include <iostream>
#include <sstream>

#include "library/external/sgp4/sgp4ext.h"   // for jday()
#include "library/external/sgp4/sgp4unit.h"  // for gstime()
#include "library/math/constants.hpp"

// Default constructor
CelestialRotation::CelestialRotation(const RotationMode rotation_mode, const std::string center_body_name) {
  planet_name_ = "Anonymous";
  rotation_mode_ = RotationMode::kIdle;
  dcm_j2000_to_xcxf_ = libra::MakeIdentityMatrix<3>();
  dcm_teme_to_xcxf_ = dcm_j2000_to_xcxf_;
  if (center_body_name == "EARTH") {
    InitCelestialRotationAsEarth(rotation_mode, center_body_name);
  } else {
    // If the center object is not defined for rotation calculation, make the DCM as a unit matrix
    std::cerr << "WARNINGS: The rotation calculation for the center object " << center_body_name;
    std::cerr << " is not supported yet." << std::endl;
    std::cerr << "The rotation matrix is set as a identity matrix" << std::endl;
    rotation_mode_ = RotationMode::kIdle;
    dcm_j2000_to_xcxf_ = libra::MakeIdentityMatrix<3>();
  }
}

// Initialize the class CelestialRotation instance as Earth
void CelestialRotation::InitCelestialRotationAsEarth(const RotationMode rotation_mode, const std::string center_body_name) {
  planet_name_ = "EARTH";
  if (center_body_name == planet_name_) {
    if (rotation_mode == RotationMode::kSimple) {
      rotation_mode_ = RotationMode::kSimple;
      // For Simple mode, we don't need initialization of the coefficients
    } else if (rotation_mode == RotationMode::kFull) {
      rotation_mode_ = RotationMode::kFull;
      // For Full mode, initialize the coefficients
      // The hard coded values are consistent with the reference document. The unit deg and rad are mixed.

      // Coefficients to compute mean obliquity of the ecliptic
      // The actual unit of the coefficients are [rad/century^i], where i is the index of the array
      c_epsilon_rad_[0] = 23.4392911 * libra::deg_to_rad;      // [rad]
      c_epsilon_rad_[1] = -46.8150000 * libra::arcsec_to_rad;  // [rad/century]
      c_epsilon_rad_[2] = -5.9000e-4 * libra::arcsec_to_rad;   // [rad/century^2]
      c_epsilon_rad_[3] = 1.8130e-3 * libra::arcsec_to_rad;    // [rad/century^3]

      // Coefficients to compute Delaunay angles
      // The actual unit of the coefficients are [rad/century^i], where i is the index of the array
      c_lm_rad_[0] = 134.96340251 * libra::deg_to_rad;            // [rad]
      c_lm_rad_[1] = 1717915923.21780000 * libra::arcsec_to_rad;  // [rad/century]
      c_lm_rad_[2] = 31.87920000 * libra::arcsec_to_rad;          // [rad/century^2]
      c_lm_rad_[3] = 0.05163500 * libra::arcsec_to_rad;           // [rad/century^3]
      c_lm_rad_[4] = -0.00024470 * libra::arcsec_to_rad;          // [rad/century^4]

      c_ls_rad_[0] = 357.52910918 * libra::deg_to_rad;           // [rad]
      c_ls_rad_[1] = 129596581.04810000 * libra::arcsec_to_rad;  // [rad/century]
      c_ls_rad_[2] = -0.55320000 * libra::arcsec_to_rad;         // [rad/century^2]
      c_ls_rad_[3] = 0.00013600 * libra::arcsec_to_rad;          // [rad/century^3]
      c_ls_rad_[4] = -0.00001149 * libra::arcsec_to_rad;         // [rad/century^4]

      c_f_rad_[0] = 93.27209062 * libra::deg_to_rad;             // [rad]
      c_f_rad_[1] = 1739527262.84780000 * libra::arcsec_to_rad;  // [rad/century]
      c_f_rad_[2] = -12.75120000 * libra::arcsec_to_rad;         // [rad/century^2]
      c_f_rad_[3] = -0.00103700 * libra::arcsec_to_rad;          // [rad/century^3]
      c_f_rad_[4] = 0.00000417 * libra::arcsec_to_rad;           // [rad/century^4]

      c_d_rad_[0] = 297.85019547 * libra::deg_to_rad;            // [rad]
      c_d_rad_[1] = 1602961601.20900000 * libra::arcsec_to_rad;  // [rad/century]
      c_d_rad_[2] = -6.37060000 * libra::arcsec_to_rad;          // [rad/century^2]
      c_d_rad_[3] = 0.00659300 * libra::arcsec_to_rad;           // [rad/century^3]
      c_d_rad_[4] = -0.00003169 * libra::arcsec_to_rad;          // [rad/century^4]

      c_o_rad_[0] = 125.04455501 * libra::deg_to_rad;          // [rad]
      c_o_rad_[1] = -6962890.54310000 * libra::arcsec_to_rad;  // [rad/century]
      c_o_rad_[2] = 7.47220000 * libra::arcsec_to_rad;         // [rad/century^2]
      c_o_rad_[3] = 0.00770200 * libra::arcsec_to_rad;         // [rad/century^3]
      c_o_rad_[4] = -0.00005939 * libra::arcsec_to_rad;        // [rad/century^4]

      // Coefficients to compute nutation angles
      c_d_epsilon_rad_[0] = 9.2050 * libra::arcsec_to_rad;   // [rad]
      c_d_epsilon_rad_[1] = 0.5730 * libra::arcsec_to_rad;   // [rad]
      c_d_epsilon_rad_[2] = -0.0900 * libra::arcsec_to_rad;  // [rad]
      c_d_epsilon_rad_[3] = 0.0980 * libra::arcsec_to_rad;   // [rad]
      c_d_epsilon_rad_[4] = 0.0070 * libra::arcsec_to_rad;   // [rad]
      c_d_epsilon_rad_[5] = -0.0010 * libra::arcsec_to_rad;  // [rad]
      c_d_epsilon_rad_[6] = 0.0220 * libra::arcsec_to_rad;   // [rad]
      c_d_epsilon_rad_[7] = 0.0130 * libra::arcsec_to_rad;   // [rad]
      c_d_epsilon_rad_[8] = -0.0100 * libra::arcsec_to_rad;  // [rad]

      c_d_psi_rad_[0] = -17.2060 * libra::arcsec_to_rad;  // [rad]
      c_d_psi_rad_[1] = -1.3170 * libra::arcsec_to_rad;   // [rad]
      c_d_psi_rad_[2] = 0.2070 * libra::arcsec_to_rad;    // [rad]
      c_d_psi_rad_[3] = -0.2280 * libra::arcsec_to_rad;   // [rad]
      c_d_psi_rad_[4] = 0.1480 * libra::arcsec_to_rad;    // [rad]
      c_d_psi_rad_[5] = 0.0710 * libra::arcsec_to_rad;    // [rad]
      c_d_psi_rad_[6] = -0.0520 * libra::arcsec_to_rad;   // [rad]
      c_d_psi_rad_[7] = -0.0300 * libra::arcsec_to_rad;   // [rad]
      c_d_psi_rad_[8] = 0.0220 * libra::arcsec_to_rad;    // [rad]

      // Coefficients to compute precession angle
      // The actual unit of the coefficients are [rad/century^i], where i is the index of the array
      c_zeta_rad_[0] = 2306.218100 * libra::arcsec_to_rad;  // [rad/century]
      c_zeta_rad_[1] = 0.301880 * libra::arcsec_to_rad;     // [rad/century^2]
      c_zeta_rad_[2] = 0.017998 * libra::arcsec_to_rad;     // [rad/century^3]

      c_theta_rad_[0] = 2004.310900 * libra::arcsec_to_rad;  // [rad/century]
      c_theta_rad_[1] = -0.426650 * libra::arcsec_to_rad;    // [rad/century^2]
      c_theta_rad_[2] = -0.041833 * libra::arcsec_to_rad;    // [rad/century^3]

      c_z_rad_[0] = 2306.218100 * libra::arcsec_to_rad;  // [rad/century]
      c_z_rad_[1] = 1.094680 * libra::arcsec_to_rad;     // [rad/century^2]
      c_z_rad_[2] = 0.018203 * libra::arcsec_to_rad;     // [rad/century^3]
    } else {
      // If the rotation mode is neither Simple nor Full, disable the rotation calculation and make the DCM a unit matrix
      rotation_mode_ = RotationMode::kIdle;
      dcm_j2000_to_xcxf_ = libra::MakeIdentityMatrix<3>();
    }
  } else {
    // If the center object is not the Earth, disable the Earth's rotation calculation and make the DCM a unit matrix
    rotation_mode_ = RotationMode::kIdle;
    dcm_j2000_to_xcxf_ = libra::MakeIdentityMatrix<3>();
  }
}

void CelestialRotation::Update(const double julian_date) {
  double gmst_rad = gstime(julian_date);  // It is a bit different with 長沢(Nagasawa)'s algorithm. TODO: Check the correctness

  if (rotation_mode_ == RotationMode::kFull) {
    // Compute Julian date for terrestrial time
    double jdTT_day = julian_date + kDtUt1Utc_ * kSec2Day_;  // TODO: Check the correctness. Problem is that S2E doesn't have Gregorian calendar.

    // Compute nth power of julian century for terrestrial time.
    // The actual unit of tTT_century is [century^(i+1)], i is the index of the array
    double terrestrial_time_julian_century[4];
    terrestrial_time_julian_century[0] = (jdTT_day - kJulianDateJ2000_) / kDayJulianCentury_;
    for (int i = 0; i < 3; i++) {
      terrestrial_time_julian_century[i + 1] = terrestrial_time_julian_century[i] * terrestrial_time_julian_century[0];
    }

    libra::Matrix<3, 3> dcm_precession;
    libra::Matrix<3, 3> dcm_nutation;
    libra::Matrix<3, 3> dcm_rotation;
    libra::Matrix<3, 3> dcm_polar_motion;
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
    dcm_j2000_to_xcxf_ = dcm_polar_motion * dcm_rotation * dcm_nutation * dcm_precession;
  } else if (rotation_mode_ == RotationMode::kSimple) {
    // In this case, only Axial Rotation is executed, with its argument replaced from G'A'ST to G'M'ST
    // FIXME: Not suitable when the center body is not the earth
    dcm_j2000_to_xcxf_ = AxialRotation(gmst_rad);
  } else {
    // Leave the DCM as unit Matrix(diag{1,1,1})
    return;
  }
}

libra::Matrix<3, 3> CelestialRotation::AxialRotation(const double gast_rad) { return libra::MakeRotationMatrixZ(gast_rad); }

libra::Matrix<3, 3> CelestialRotation::Nutation(const double (&t_tt_century)[4]) {
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
  libra::Matrix<3, 3> x_epsi_1st = libra::MakeRotationMatrixX(epsilon_rad_);
  libra::Matrix<3, 3> z_d_psi = libra::MakeRotationMatrixZ(-d_psi_rad_);
  libra::Matrix<3, 3> x_epsi_2nd = libra::MakeRotationMatrixX(-epsi_mod_rad);

  libra::Matrix<3, 3> dcm_nutation;
  dcm_nutation = x_epsi_2nd * z_d_psi * x_epsi_1st;

  return dcm_nutation;
}

libra::Matrix<3, 3> CelestialRotation::Precession(const double (&t_tt_century)[4]) {
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
  libra::Matrix<3, 3> z_zeta = libra::MakeRotationMatrixZ(-zeta_rad);
  libra::Matrix<3, 3> y_theta = libra::MakeRotationMatrixY(theta_rad);
  libra::Matrix<3, 3> z_z = libra::MakeRotationMatrixZ(-z_rad);

  libra::Matrix<3, 3> dcm_precession;
  dcm_precession = z_z * y_theta * z_zeta;

  return dcm_precession;
}

libra::Matrix<3, 3> CelestialRotation::PolarMotion(const double x_p, const double y_p) {
  libra::Matrix<3, 3> dcm_polar_motion;

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
