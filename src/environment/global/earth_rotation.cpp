/**
 * @file earth_rotation.cpp
 * @brief Class to calculate the earth rotation
 * @note Refs: 福島,"天体の回転運動理論入門講義ノート", 2007 (in Japanese),
 *             長沢,"天体の位置計算(増補版)", 2001 (in Japanese),
 *             IERS Conventions 2003
 */

#include "earth_rotation.hpp"

#include "physical_constants.hpp"

#include <cmath>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <iostream>
#include <sstream>

#include "library/external/sgp4/sgp4ext.h"   // for jday()
#include "library/external/sgp4/sgp4unit.h"  // for gstime()
#include "library/math/constants.hpp"

// Default constructor
EarthRotation::EarthRotation(const EarthRotationMode rotation_mode) : rotation_mode_(rotation_mode) {
  dcm_j2000_to_ecef_ = libra::MakeIdentityMatrix<3>();
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
    dcm_j2000_to_ecef_ = libra::MakeIdentityMatrix<3>();
  }
}

namespace {

double CalcGreenwichMeanSiderealTimeRad(const long double julian_date_ut1) {
  const long double twopi = 2.0L * libra::pi_v<long double>;
  const long double deg2rad = libra::pi_v<long double> / 180.0L;
  const long double tut1 = (julian_date_ut1 - 2451545.0L) / 36525.0L;

  long double temp_sec =
      -6.2e-6L * tut1 * tut1 * tut1 + 0.093104L * tut1 * tut1 + (876600.0L * 3600.0L + 8640184.812866L) * tut1 + 67310.54841L;
  long double gmst_rad = std::fmod(temp_sec * deg2rad / 240.0L, twopi);
  if (gmst_rad < 0.0L) gmst_rad += twopi;

  return static_cast<double>(gmst_rad);
}

}  // namespace

void EarthRotation::LoadEarthOrientation(const std::string& path) {
  std::ifstream file(path);
  if (!file) throw std::runtime_error("Cannot open Earth orientation data: " + path);
  auto records = std::make_shared<std::vector<EarthOrientation>>();
  std::string line;
  while (std::getline(file, line)) {
    // IERS finals2000A fixed columns: MJD 8-15, UT1 flag 58, DUT1 59-68 (1-based).
    // The file may end with dates whose EOP values have not yet been published.
    if (line.size() < 68 || (line[57] != 'I' && line[57] != 'P')) continue;
    const double mjd = std::stod(line.substr(7, 8));
    const double dut1 = std::stod(line.substr(58, 10));
    if (!std::isfinite(mjd) || !std::isfinite(dut1) ||
        (!records->empty() && mjd - records->back().mjd_utc != 1.0))
      throw std::runtime_error("Invalid daily Earth orientation data: " + path);
    records->push_back({mjd, dut1});
  }
  if (records->size() < 2) throw std::runtime_error("No daily DUT1 records: " + path);
  earth_orientation_ = records;
}

double EarthRotation::GetUt1MinusUtc(const double mjd_utc) const {
  if (!earth_orientation_) throw std::runtime_error("Earth orientation data is not loaded");
  const auto& records = *earth_orientation_;
  if (!std::isfinite(mjd_utc) || mjd_utc < records.front().mjd_utc || mjd_utc > records.back().mjd_utc)
    throw std::runtime_error("UTC epoch is outside finals2000A.all coverage");
  const auto next = std::upper_bound(records.begin(), records.end(), mjd_utc,
      [](double mjd, const EarthOrientation& record) { return mjd < record.mjd_utc; });
  if (next == records.end()) return records.back().dut1_s;
  const auto& previous = *(next - 1);
  double delta = next->dut1_s - previous.dut1_s;
  // DUT1 jumps by a whole second at a leap second. Interpolate its continuous
  // part within the UTC day, then adopt the new offset at the next midnight.
  delta -= std::round(delta);
  return previous.dut1_s + (mjd_utc - previous.mjd_utc) * delta;
}

void EarthRotation::Update(const double julian_date) {
  UpdateWithElapsedTime(julian_date, 0.0);
}

void EarthRotation::UpdateWithElapsedTime(const double start_julian_date, const double elapsed_time_s) {
  const long double julian_date = static_cast<long double>(start_julian_date) + static_cast<long double>(elapsed_time_s) * kSec2Day_;
  if (rotation_mode_ == EarthRotationMode::kIdle) return;
  const double dut1_s = GetUt1MinusUtc(static_cast<double>(julian_date - 2400000.5L));
  // Axial rotation uses UT1; precession and nutation use TT. Keep UTC unchanged.
  const long double julian_date_ut1 = julian_date + static_cast<long double>(dut1_s) * kSec2Day_;
  double gmst_rad = CalcGreenwichMeanSiderealTimeRad(julian_date_ut1);

  if (rotation_mode_ == EarthRotationMode::kFull) {
    const long double tt_minus_utc_s = environment::tt_minus_tai_s_v<long double> +
        environment::tai_minus_gps_s_v<long double> + environment::gps_minus_utc_s_v<long double>;
    const long double terrestrial_time_julian_day = julian_date + tt_minus_utc_s * kSec2Day_;

    double terrestrial_time_julian_century[4];
    terrestrial_time_julian_century[0] =
        static_cast<double>((terrestrial_time_julian_day - static_cast<long double>(kJulianDateJ2000_)) / kDayJulianCentury_);
    for (int i = 0; i < 3; i++) {
      terrestrial_time_julian_century[i + 1] = terrestrial_time_julian_century[i] * terrestrial_time_julian_century[0];
    }

    libra::Matrix<3, 3> dcm_precession;
    libra::Matrix<3, 3> dcm_nutation;
    libra::Matrix<3, 3> dcm_rotation;
    libra::Matrix<3, 3> dcm_polar_motion;
    dcm_precession = Precession(terrestrial_time_julian_century);
    dcm_nutation = Nutation(terrestrial_time_julian_century);

    double equinox_rad = d_psi_rad_ * cos(epsilon_rad_ + d_epsilon_rad_);
    double gast_rad = gmst_rad + equinox_rad;
    dcm_rotation = AxialRotation(gast_rad);
    double x_p = 0.0;
    double y_p = 0.0;
    dcm_polar_motion = PolarMotion(x_p, y_p);

    dcm_j2000_to_ecef_ = dcm_polar_motion * dcm_rotation * dcm_nutation * dcm_precession;
  } else if (rotation_mode_ == EarthRotationMode::kSimple) {
    dcm_j2000_to_ecef_ = AxialRotation(gmst_rad);
  }
}

libra::Matrix<3, 3> EarthRotation::AxialRotation(const double gast_rad) { return libra::MakeRotationMatrixZ(gast_rad); }

libra::Matrix<3, 3> EarthRotation::Nutation(const double (&t_tt_century)[4]) {
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

libra::Matrix<3, 3> EarthRotation::Precession(const double (&t_tt_century)[4]) {
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

libra::Matrix<3, 3> EarthRotation::PolarMotion(const double x_p, const double y_p) {
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
