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

math::Matrix<6, 6> CalcStateTransformationMatrixLvlhToTschaunerHampel(const double gravity_constant_m3_s2, const double eccentricity,
                                                                      const double angular_momentum_kg_m2_s, const double true_anomaly_rad) {
  math::Matrix<6, 6> transition_matrix;
  transition_matrix[0][0] = gravity_constant_m3_s2 * (1.0 + eccentricity * cos(true_anomaly_rad)) / pow(angular_momentum_kg_m2_s, 1.5);
  transition_matrix[0][1] = 0.0;
  transition_matrix[0][2] = 0.0;
  transition_matrix[0][3] = 0.0;
  transition_matrix[0][4] = 0.0;
  transition_matrix[0][5] = 0.0;
  transition_matrix[1][0] = 0.0;
  transition_matrix[1][1] = gravity_constant_m3_s2 * (1.0 + eccentricity * cos(true_anomaly_rad)) / pow(angular_momentum_kg_m2_s, 1.5);
  transition_matrix[1][2] = 0.0;
  transition_matrix[1][3] = 0.0;
  transition_matrix[1][4] = 0.0;
  transition_matrix[1][5] = 0.0;
  transition_matrix[2][0] = 0.0;
  transition_matrix[2][1] = 0.0;
  transition_matrix[2][2] = gravity_constant_m3_s2 * (1.0 + eccentricity * cos(true_anomaly_rad)) / pow(angular_momentum_kg_m2_s, 1.5);
  transition_matrix[2][3] = 0.0;
  transition_matrix[2][4] = 0.0;
  transition_matrix[2][5] = 0.0;
  transition_matrix[3][0] = -gravity_constant_m3_s2 * eccentricity * sin(true_anomaly_rad) / pow(angular_momentum_kg_m2_s, 1.5);
  transition_matrix[3][1] = 0.0;
  transition_matrix[3][2] = 0.0;
  transition_matrix[3][3] = pow(angular_momentum_kg_m2_s, 1.5) / gravity_constant_m3_s2 / (1.0 + eccentricity * cos(true_anomaly_rad));
  transition_matrix[3][4] = 0.0;
  transition_matrix[3][5] = 0.0;
  transition_matrix[4][0] = 0.0;
  transition_matrix[4][1] = -gravity_constant_m3_s2 * eccentricity * sin(true_anomaly_rad) / pow(angular_momentum_kg_m2_s, 1.5);
  transition_matrix[4][2] = 0.0;
  transition_matrix[4][3] = 0.0;
  transition_matrix[4][4] = pow(angular_momentum_kg_m2_s, 1.5) / gravity_constant_m3_s2 / (1.0 + eccentricity * cos(true_anomaly_rad));
  transition_matrix[4][5] = 0.0;
  transition_matrix[5][0] = 0.0;
  transition_matrix[5][1] = 0.0;
  transition_matrix[5][2] = -gravity_constant_m3_s2 * eccentricity * sin(true_anomaly_rad) / pow(angular_momentum_kg_m2_s, 1.5);
  transition_matrix[5][3] = 0.0;
  transition_matrix[5][4] = 0.0;
  transition_matrix[5][5] = pow(angular_momentum_kg_m2_s, 1.5) / gravity_constant_m3_s2 / (1.0 + eccentricity * cos(true_anomaly_rad));

  return transition_matrix;
}

math::Matrix<6, 6> CalcStateTransformationMatrixTschaunerHampelToLvlh(const double gravity_constant_m3_s2, const double eccentricity,
                                                                      const double angular_momentum_kg_m2_s, const double true_anomaly_rad) {
  math::Matrix<6, 6> transition_matrix;
  transition_matrix[0][0] = pow(angular_momentum_kg_m2_s, 1.5) / gravity_constant_m3_s2 / (1.0 + eccentricity * cos(true_anomaly_rad));
  transition_matrix[0][1] = 0.0;
  transition_matrix[0][2] = 0.0;
  transition_matrix[0][3] = 0.0;
  transition_matrix[0][4] = 0.0;
  transition_matrix[0][5] = 0.0;
  transition_matrix[1][0] = 0.0;
  transition_matrix[1][1] = pow(angular_momentum_kg_m2_s, 1.5) / gravity_constant_m3_s2 / (1.0 + eccentricity * cos(true_anomaly_rad));
  transition_matrix[1][2] = 0.0;
  transition_matrix[1][3] = 0.0;
  transition_matrix[1][4] = 0.0;
  transition_matrix[1][5] = 0.0;
  transition_matrix[2][0] = 0.0;
  transition_matrix[2][1] = 0.0;
  transition_matrix[2][2] = pow(angular_momentum_kg_m2_s, 1.5) / gravity_constant_m3_s2 / (1.0 + eccentricity * cos(true_anomaly_rad));
  transition_matrix[2][3] = 0.0;
  transition_matrix[2][4] = 0.0;
  transition_matrix[2][5] = 0.0;
  transition_matrix[3][0] = gravity_constant_m3_s2 * eccentricity * sin(true_anomaly_rad) / pow(angular_momentum_kg_m2_s, 1.5);
  transition_matrix[3][1] = 0.0;
  transition_matrix[3][2] = 0.0;
  transition_matrix[3][3] = gravity_constant_m3_s2 * (1.0 + eccentricity * cos(true_anomaly_rad)) / pow(angular_momentum_kg_m2_s, 1.5);
  transition_matrix[3][4] = 0.0;
  transition_matrix[3][5] = 0.0;
  transition_matrix[4][0] = 0.0;
  transition_matrix[4][1] = gravity_constant_m3_s2 * eccentricity * sin(true_anomaly_rad) / pow(angular_momentum_kg_m2_s, 1.5);
  transition_matrix[4][2] = 0.0;
  transition_matrix[4][3] = 0.0;
  transition_matrix[4][4] = gravity_constant_m3_s2 * (1.0 + eccentricity * cos(true_anomaly_rad)) / pow(angular_momentum_kg_m2_s, 1.5);
  transition_matrix[4][5] = 0.0;
  transition_matrix[5][0] = 0.0;
  transition_matrix[5][1] = 0.0;
  transition_matrix[5][2] = gravity_constant_m3_s2 * eccentricity * sin(true_anomaly_rad) / pow(angular_momentum_kg_m2_s, 1.5);
  transition_matrix[5][3] = 0.0;
  transition_matrix[5][4] = 0.0;
  transition_matrix[5][5] = gravity_constant_m3_s2 * (1.0 + eccentricity * cos(true_anomaly_rad)) / pow(angular_momentum_kg_m2_s, 1.5);

  return transition_matrix;
}

}  // namespace orbit
