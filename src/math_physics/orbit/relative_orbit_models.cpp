/**
 * @file relative_orbit_models.cpp
 * @brief Functions for relative orbit
 */
#include "relative_orbit_models.hpp"

#include <environment/global/physical_constants.hpp>

#include "./sgp4/sgp4unit.h"  // for getgravconst()

namespace orbit {

math::Matrix<6, 6> CalcHillSystemMatrix(double orbit_radius_m, double gravity_constant_m3_s2) {
  math::Matrix<6, 6> system_matrix;

  double n = sqrt(gravity_constant_m3_s2 / pow(orbit_radius_m, 3));
  system_matrix[0][0] = 0.0;
  system_matrix[0][1] = 0.0;
  system_matrix[0][2] = 0.0;
  system_matrix[0][3] = 1.0;
  system_matrix[0][4] = 0.0;
  system_matrix[0][5] = 0.0;
  system_matrix[1][0] = 0.0;
  system_matrix[1][1] = 0.0;
  system_matrix[1][2] = 0.0;
  system_matrix[1][3] = 0.0;
  system_matrix[1][4] = 1.0;
  system_matrix[1][5] = 0.0;
  system_matrix[2][0] = 0.0;
  system_matrix[2][1] = 0.0;
  system_matrix[2][2] = 0.0;
  system_matrix[2][3] = 0.0;
  system_matrix[2][4] = 0.0;
  system_matrix[2][5] = 1.0;
  system_matrix[3][0] = 3.0 * n * n;
  system_matrix[3][1] = 0.0;
  system_matrix[3][2] = 0.0;
  system_matrix[3][3] = 0.0;
  system_matrix[3][4] = 2.0 * n;
  system_matrix[3][5] = 0.0;
  system_matrix[4][0] = 0.0;
  system_matrix[4][1] = 0.0;
  system_matrix[4][2] = 0.0;
  system_matrix[4][3] = -2.0 * n;
  system_matrix[4][4] = 0.0;
  system_matrix[4][5] = 0.0;
  system_matrix[5][0] = 0.0;
  system_matrix[5][1] = 0.0;
  system_matrix[5][2] = -n * n;
  system_matrix[5][3] = 0.0;
  system_matrix[5][4] = 0.0;
  system_matrix[5][5] = 0.0;

  return system_matrix;
}

math::Matrix<6, 6> CalcHcwStm(double orbit_radius_m, double gravity_constant_m3_s2, double elapsed_time_s) {
  math::Matrix<6, 6> stm;

  double n = sqrt(gravity_constant_m3_s2 / pow(orbit_radius_m, 3));
  double t = elapsed_time_s;
  stm[0][0] = 4.0 - 3.0 * cos(n * t);
  stm[0][1] = 0.0;
  stm[0][2] = 0.0;
  stm[0][3] = sin(n * t) / n;
  stm[0][4] = 2.0 / n - 2.0 * cos(n * t) / n;
  stm[0][5] = 0.0;
  stm[1][0] = -6.0 * n * t + 6 * sin(n * t);
  stm[1][1] = 1.0;
  stm[1][2] = 0.0;
  stm[1][3] = -2.0 / n + 2.0 * cos(n * t) / n;
  stm[1][4] = 4.0 * sin(n * t) / n - 3.0 * t;
  stm[1][5] = 0.0;
  stm[2][0] = 0.0;
  stm[2][1] = 0.0;
  stm[2][2] = cos(n * t);
  stm[2][3] = 0.0;
  stm[2][4] = 0.0;
  stm[2][5] = sin(n * t) / n;
  stm[3][0] = 3.0 * n * sin(n * t);
  stm[3][1] = 0.0;
  stm[3][2] = 0.0;
  stm[3][3] = cos(n * t);
  stm[3][4] = 2 * sin(n * t);
  stm[3][5] = 0.0;
  stm[4][0] = -6.0 * n + 6.0 * n * cos(n * t);
  stm[4][1] = 0.0;
  stm[4][2] = 0.0;
  stm[4][3] = -2.0 * sin(n * t);
  stm[4][4] = -3.0 + 4.0 * cos(n * t);
  stm[4][5] = 0.0;
  stm[5][0] = 0.0;
  stm[5][1] = 0.0;
  stm[5][2] = -n * sin(n * t);
  stm[5][3] = 0.0;
  stm[5][4] = 0.0;
  stm[5][5] = cos(n * t);
  return stm;
}

// 定数やreferenceの軌道要素は以下のように取得できます
// gravconsttype whichconst = wgs72;
// double mu_km3_s2, radius_earth_km, tumin, xke, j2, j3, j4, j3oj2;
// getgravconst(whichconst, tumin, mu_km3_s2, radius_earth_km, xke, j2, j3, j4, j3oj2);
// double semi_major_axis_m = reference_oe->GetSemiMajorAxis_m();
// double eccentricity = reference_oe->GetEccentricity();

math::Matrix<6, 6> CalcMeltonStm(double orbit_radius_m, double gravity_constant_m3_s2, double elapsed_time_s, OrbitalElements* reference_oe) {
  math::Matrix<6, 6> stm;
  // ここでstmを計算してください
  return stm;
}

math::Matrix<6, 6> CalcSsStm(double orbit_radius_m, double gravity_constant_m3_s2, double elapsed_time_s, OrbitalElements* reference_oe) {
  math::Matrix<6, 6> stm;
  // ここでstmを計算してください
  return stm;
}

math::Vector<6> CalcSsCorrectionTerm(double orbit_radius_m, double gravity_constant_m3_s2, double elapsed_time_s, OrbitalElements* reference_oe) {
  math::Vector<6> correction_term;
  // ここでstmを計算してください
  return correction_term;
}

math::Matrix<6, 6> CalcSabatiniStm(double orbit_radius_m, double gravity_constant_m3_s2, double elapsed_time_s, OrbitalElements* reference_oe) {
  math::Matrix<6, 6> stm;
  // ここでstmを計算してください
  return stm;
}

math::Matrix<6, 6> CalcCarterStm(double orbit_radius_m, double gravity_constant_m3_s2, double f_ref_rad, OrbitalElements* reference_oe) {
  math::Matrix<6, 6> stm;
  // ここでstmを計算してください
  double n = sqrt(gravity_constant_m3_s2 / pow(orbit_radius_m, 3));
  double e = reference_oe->GetEccentricity();
  double E_rad = 2 * atan(sqrt((1 - e) / (1 + e)) * tan(f_ref_rad / 2));
  double k = e * cos(f_ref_rad) + 1;
  double K1 = pow(1 - e * e, -2.5) * (-1.5 * e * E_rad + (1 + e * e) * sin(E_rad) - e * sin(2. * E_rad) / 4.);
  double K2 = pow(1 - e * e, -2.5) * (0.5 * E_rad - 0.25 * sin(2 * E_rad) - e * pow(sin(E_rad), 3) / 3.);
  double phi1 = sin(f_ref_rad) * k;
  double phi2 = 2 * e * phi1 * (sin(f_ref_rad) / pow(k, 3) - K2) - cos(f_ref_rad) / k;
  double phi3 = (6 * e * phi1 * K2 - (2 * pow(sin(f_ref_rad), 2) / pow(k, 2)) - (pow(cos(f_ref_rad), 2) / k) - pow(cos(f_ref_rad), 2));
  double phi1_prime = cos(f_ref_rad) * k - e * pow(sin(f_ref_rad), 2);
  double sigma4 = atan(tan(f_ref_rad / 2) * sqrt(-(e - 1) / (e + 1)));
  double term1_phi2_prime = sin(f_ref_rad) / k;
  double term2_phi2_prime = 2 * e * sin(f_ref_rad) * k * (cos(f_ref_rad) / pow(k, 3)) - (cos(f_ref_rad) / pow(1 - pow(e, 2), 5.0 / 2.0));
  double phi2_prime = term1_phi2_prime + term2_phi2_prime;
  double phi3_prime = k - 4 * e * pow(sin(f_ref_rad), 3) / pow(E_rad / 2., 3) + 2 * cos(f_ref_rad) * sin(f_ref_rad) / pow(E_rad / 2., 2) - 6 * e * cos(f_ref_rad) * E_rad / 2. / pow(1 - pow(e, 2), 5.0 / 2.0);
  double S_phi1 = -cos(f_ref_rad) - (e / 2.0) * pow(cos(f_ref_rad), 2);
  double S_phi2 = 3 * e * k * k * K2 - (sin(f_ref_rad) / k);
  double S_phi3 = -6 * k * k * K2 - ((2 + k) / (2 * k)) * sin(2 * f_ref_rad); 
  stm[0][0] = 0.0;
  stm[0][1] = 0.0;
  stm[0][2] = 0.0;
  stm[0][3] = 0.0;
  stm[0][4] = 0.0;
  stm[0][5] = 0.0;
  stm[1][0] = 0.0;
  stm[1][1] = 0.0;
  stm[1][2] = 0.0;
  stm[1][3] = 0.0;
  stm[1][4] = 0.0;
  stm[1][5] = 0.0;
  stm[2][0] = 0.0;
  stm[2][1] = 0.0;
  stm[2][2] = 0.0;
  stm[2][3] = 0.0;
  stm[2][4] = 0.0;
  stm[2][5] = 0.0;
  stm[3][0] = 0.0;
  stm[3][1] = 0.0;
  stm[3][2] = 0.0;
  stm[3][3] = 0.0;
  stm[3][4] = 0.0;
  stm[3][5] = 0.0;
  stm[4][0] = 0.0;
  stm[4][1] = 0.0;
  stm[4][2] = 0.0;
  stm[4][3] = 0.0;
  stm[4][4] = 0.0;
  stm[4][5] = 0.0;
  stm[5][0] = 0.0;
  stm[5][1] = 0.0;
  stm[5][2] = 0.0;
  stm[5][3] = 0.0;
  stm[5][4] = 0.0;
  stm[5][5] = 0.0;
  return stm;
}

}  // namespace orbit
