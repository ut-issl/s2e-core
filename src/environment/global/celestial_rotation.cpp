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
CelestialRotation::CelestialRotation(const RotationMode rotation_mode, const std::string center_obj) {
  planet_name_ = "Anonymous";
  rotation_mode_ = Idle;
  unitalize(dcm_j2000_to_xcxf_);
  dcm_teme_to_xcxf_ = dcm_j2000_to_xcxf_;
  if (center_obj == "EARTH") {
    Init_CelestialRotation_As_Earth(rotation_mode, center_obj);
  }
}

// Initialize the class CelestialRotation instance as Earth
void CelestialRotation::Init_CelestialRotation_As_Earth(const RotationMode rotation_mode, const std::string center_obj) {
  planet_name_ = "EARTH";
  if (center_obj == planet_name_) {
    if (rotation_mode == Simple) {
      rotation_mode_ = Simple;
      // For Simple mode, we don't need initialization of the coefficients
    } else if (rotation_mode == Full) {
      rotation_mode_ = Full;
      // For Full mode, initialize the coefficients
      // The hard coded values are consistent with the reference document. The unit deg and rad are mixed.

      // Coefficients to compute mean obliquity of the ecliptic
      // The actual unit of the coefficients are [rad/century^i], where i is the index of the array
      c_epsilon_rad_[0] = 23.4392911 * libra::deg_to_rad;      // [rad]
      c_epsilon_rad_[1] = -46.8150000 * libra::arcsec_to_rad;  // [rad/century]
      c_epsilon_rad_[2] = -5.9000e-4 * libra::arcsec_to_rad;   // [rad/century^2]
      c_epsilon_rad_[3] = 1.8130e-3 * libra::arcsec_to_rad;    // [rad/century^3]

      // Coefficients to compute delauney angles
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
      rotation_mode_ = Idle;
      unitalize(dcm_j2000_to_xcxf_);
    }
  } else {
    // If the center object is not the Earth, disable the Earth's rotation calculation and make the DCM a unit matrix
    rotation_mode_ = Idle;
    unitalize(dcm_j2000_to_xcxf_);
  }
}

void CelestialRotation::Update(const double JulianDate) {
  double gmst_rad = gstime(JulianDate);  // It is a bit different with 長沢(Nagasawa)'s algorithm. TODO: Check the correctness

  if (rotation_mode_ == Full) {
    // Compute Julian date for terestrial time
    double jdTT_day = JulianDate + kDtUt1Utc_ * kSec2Day_;  // TODO: Check the correctness. Problem is thtat S2E doesn't have Gregorian calendar.

    // Compute nth power of julian century for terrestrial time the actual unit of tTT_century is [century^(i+1)], i is the index of the array
    double tTT_century[4];
    tTT_century[0] = (jdTT_day - kJulianDateJ2000_) / kDayJulianCentury_;
    for (int i = 0; i < 3; i++) {
      tTT_century[i + 1] = tTT_century[i] * tTT_century[0];
    }

    libra::Matrix<3, 3> P;
    libra::Matrix<3, 3> N;
    libra::Matrix<3, 3> R;
    libra::Matrix<3, 3> W;
    // Nutation + Precession
    P = Precession(tTT_century);
    N = Nutation(tTT_century);  // epsilon_rad_, d_epsilon_rad_, d_psi_rad_ are
                                // updated in this proccedure

    // Axial Rotation
    double Eq_rad = d_psi_rad_ * cos(epsilon_rad_ + d_epsilon_rad_);  // Equation of equinoxes [rad]
    double gast_rad = gmst_rad + Eq_rad;                              // Greenwitch 'Appearent' Sidereal Time [rad]
    R = AxialRotation(gast_rad);
    // Polar motion (isnot considered so far, even without polar motion, the result agrees well with the matlab reference)
    double Xp = 0.0;
    double Yp = 0.0;
    W = PolarMotion(Xp, Yp);

    // Total orientation
    dcm_j2000_to_xcxf_ = W * R * N * P;
  } else if (rotation_mode_ == Simple) {
    // In this case, only Axial Rotation is executed, with its argument replaced from G'A'ST to G'M'ST
    dcm_j2000_to_xcxf_ = AxialRotation(gmst_rad);
  } else {
    // Leave the DCM as unit Matrix(diag{1,1,1})
    return;
  }
}

libra::Matrix<3, 3> CelestialRotation::AxialRotation(const double GAST_rad) { return libra::rotz(GAST_rad); }

libra::Matrix<3, 3> CelestialRotation::Nutation(const double (&tTT_century)[4]) {
  // Mean obliquity of the ecliptic
  epsilon_rad_ = c_epsilon_rad_[0];
  for (int i = 0; i < 3; i++) {
    epsilon_rad_ += c_epsilon_rad_[i + 1] * tTT_century[i];
  }

  // Compute five delauney angles(l=lm,l'=ls,F,D,Ω=O)
  // Mean anomaly of the moon
  double lm_rad = c_lm_rad_[0];
  for (int i = 0; i < 4; i++) {
    lm_rad += c_lm_rad_[i + 1] * tTT_century[i];
  }
  // Mean anomaly of the sun
  double ls_rad = c_ls_rad_[0];
  for (int i = 0; i < 4; i++) {
    ls_rad += c_ls_rad_[i + 1] * tTT_century[i];
  }
  // Mean longitude of the moon - mean longitude of ascending node of the moon
  double F_rad = c_f_rad_[0];
  for (int i = 0; i < 4; i++) {
    F_rad += c_f_rad_[i + 1] * tTT_century[i];
  }
  // Mean elogation of the moon from the sun
  double D_rad = c_d_rad_[0];
  for (int i = 0; i < 4; i++) {
    D_rad += c_d_rad_[i + 1] * tTT_century[i];
  }
  // Mean longitude of ascending node of the moon
  double O_rad = c_o_rad_[0];
  for (int i = 0; i < 4; i++) {
    O_rad += c_o_rad_[i + 1] * tTT_century[i];
  }

  // Additional angles
  double L_rad = F_rad + O_rad;   // F + Ω     [rad]
  double Ld_rad = L_rad - D_rad;  // F - D + Ω [rad]

  // Compute luni-solar nutation
  // Nutation in obliquity
  d_psi_rad_ = c_d_psi_rad_[0] * sin(O_rad) + c_d_psi_rad_[1] * sin(2 * Ld_rad) + c_d_psi_rad_[2] * sin(2 * O_rad) +
               c_d_psi_rad_[3] * sin(2 * L_rad) + c_d_psi_rad_[4] * sin(ls_rad);  // [rad]
  d_psi_rad_ = d_psi_rad_ + c_d_psi_rad_[5] * sin(lm_rad) + c_d_psi_rad_[6] * sin(2 * Ld_rad + ls_rad) + c_d_psi_rad_[7] * sin(2 * L_rad + lm_rad) +
               c_d_psi_rad_[8] * sin(2 * Ld_rad - ls_rad);  // [rad]

  // Nutation in longitude
  d_epsilon_rad_ = c_d_epsilon_rad_[0] * cos(O_rad) + c_d_epsilon_rad_[1] * cos(2 * Ld_rad) + c_d_epsilon_rad_[2] * cos(2 * O_rad) +
                   c_d_epsilon_rad_[3] * cos(2 * L_rad) + c_d_epsilon_rad_[4] * cos(ls_rad);  // [rad]
  d_epsilon_rad_ = d_epsilon_rad_ + c_d_epsilon_rad_[5] * cos(lm_rad) + c_d_epsilon_rad_[6] * cos(2 * Ld_rad + ls_rad) +
                   c_d_epsilon_rad_[7] * cos(2 * L_rad + lm_rad) + c_d_epsilon_rad_[8] * cos(2 * Ld_rad - ls_rad);  // [rad]

  double epsi_mod_rad = epsilon_rad_ + d_epsilon_rad_;
  libra::Matrix<3, 3> X_epsi_1st = libra::rotx(epsilon_rad_);
  libra::Matrix<3, 3> Z_dpsi = libra::rotz(-d_psi_rad_);
  libra::Matrix<3, 3> X_epsi_2nd = libra::rotx(-epsi_mod_rad);

  libra::Matrix<3, 3> N;
  N = X_epsi_2nd * Z_dpsi * X_epsi_1st;

  return N;
}

libra::Matrix<3, 3> CelestialRotation::Precession(const double (&tTT_century)[4]) {
  // Compute precession angles(zeta, theta, z)
  double zeta_rad = 0.0;
  for (int i = 0; i < 3; i++) {
    zeta_rad += c_zeta_rad_[i] * tTT_century[i];
  }
  double theta_rad = 0.0;
  for (int i = 0; i < 3; i++) {
    theta_rad += c_theta_rad_[i] * tTT_century[i];
  }
  double z_rad = 0.0;
  for (int i = 0; i < 3; i++) {
    z_rad += c_z_rad_[i] * tTT_century[i];
  }

  // Develop transformation matrix
  libra::Matrix<3, 3> Z_zeta = libra::rotz(-zeta_rad);
  libra::Matrix<3, 3> Y_theta = libra::roty(theta_rad);
  libra::Matrix<3, 3> Z_z = libra::rotz(-z_rad);

  libra::Matrix<3, 3> P;
  P = Z_z * Y_theta * Z_zeta;

  return P;
}

libra::Matrix<3, 3> CelestialRotation::PolarMotion(const double Xp, const double Yp) {
  libra::Matrix<3, 3> W;

  W[0][0] = 1.0;
  W[0][1] = 0.0;
  W[0][2] = -Xp;
  W[1][0] = 0.0;
  W[1][1] = 1.0;
  W[1][2] = -Yp;
  W[2][0] = Xp;
  W[2][1] = Yp;
  W[2][2] = 1.0;

  return W;
}
