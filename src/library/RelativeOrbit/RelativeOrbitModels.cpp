/**
 * @file RelativeOrbitModels.cpp
 * @brief Functions for relative orbit
 */
#include "RelativeOrbitModels.h"

libra::Matrix<6, 6> CalculateHillSystemMatrix(double orbit_radius, double mu) {
  libra::Matrix<6, 6> system_matrix;

  double n = sqrt(mu / pow(orbit_radius, 3));
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

libra::Matrix<6, 6> CalculateHCWSTM(double orbit_radius, double mu, double elapsed_sec) {
  libra::Matrix<6, 6> stm;

  double n = sqrt(mu / pow(orbit_radius, 3));
  double t = elapsed_sec;
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
