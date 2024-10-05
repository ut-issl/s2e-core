/**
 * @file relative_orbit_sabatini.cpp
 * @brief Functions to calculate Yamanaka-Ankersen STM for relative orbit
 */
#include "relative_orbit_sabatini.hpp"

#include <environment/global/physical_constants.hpp>



namespace orbit {

RelativeOrbitSabatini::RelativeOrbitSabatini(math::Vector<3> position_i_m, math::Vector<3> velocity_i_m_s) {
  initial_orbit_radius_m_ = position_i_m.CalcNorm();
  initial_angular_momentum_m2_s_ = OuterProduct(position_i_m, velocity_i_m_s).CalcNorm();
}

RelativeOrbitSabatini::~RelativeOrbitSabatini() {}

math::Matrix<6, 6> RelativeOrbitSabatini::CalcSabatiniSystemMatrix(double gravity_constant_m3_s2, double f_ref_rad, OrbitalElements* reference_oe) {
  const double mu = gravity_constant_m3_s2;

  const double j2 = 0.001082616;
  const double Re = 6378.137e3;

  double r_0 = initial_orbit_radius_m_;
  double h_0 = initial_angular_momentum_m2_s_;

  double theta = reference_oe->GetArgPerigee_rad() + f_ref_rad;
  double i = reference_oe->GetInclination_rad();

  double n = sqrt(mu / pow(r_0, 3));
  double X = 3 * j2 * pow(Re, 2) / (2 * r_0) * (1.0 / 3.0 * pow(sin(i), 2) * pow(cos(theta), 2) + 1.0 / 3.0 * pow(sin(i), 2) - 1.0 + (1.0 - 2.0 / 3.0 * pow(sin(i), 2)) * cos(theta));
  double r = r_0 + X;
  double f_h = -3.0 / 2.0 * j2 * mu * pow(Re, 2) / pow(r, 4) * sin(theta) * sin(2 * i);
  double h = h_0 + 3.0 / 4.0 * mu * j2 * pow(Re, 2) / (r_0 * h_0) * pow(sin(i), 2) * (cos(2 * theta) - 1);
  double w_x = r / h * f_h;
  double w_z = h / pow(r, 2);
  double h_dot = -3.0 / 2.0 * j2 * mu * pow(Re, 2) / pow(r, 3) * sin(2 * theta) * pow(sin(i), 2);
  double r_dot = 3.0 * j2 * pow(Re, 2) / (2 * r_0) * (-1.0 / 3.0 * n * pow(sin(i), 2) * sin(2 * theta) - n * (1 - 2.0 / 3.0 * pow(sin(i), 2)) * sin(theta));
  double w_x_dot = -3 / 2 * j2 * mu * pow(Re, 2) * sin(2 * i) * (n * h * pow(r, 3) * cos(theta) - (h_dot * pow(r, 3) + 3 * h * pow(r, 2) * r_dot) * sin(theta)) / (pow(h, 2) * pow(r, 6));
  double w_z_dot = (h_dot * r - 2 * h * r_dot) / pow(r, 3);
  double K = (6.0 * j2 * mu * pow(Re, 2)) / pow(r, 5);

  math::Matrix<6, 6> system_matrix;

  system_matrix[0][0] = 0;
  system_matrix[0][1] = 0;
  system_matrix[0][2] = 0;
  system_matrix[0][3] = 1;
  system_matrix[0][4] = 0;
  system_matrix[0][5] = 0;
  system_matrix[1][0] = 0;
  system_matrix[1][1] = 0;
  system_matrix[1][2] = 0;
  system_matrix[1][3] = 0;
  system_matrix[1][4] = 1;
  system_matrix[1][5] = 0;
  system_matrix[2][0] = 0;
  system_matrix[2][1] = 0;
  system_matrix[2][2] = 0;
  system_matrix[2][3] = 0;
  system_matrix[2][4] = 0;
  system_matrix[2][5] = 1;
  system_matrix[3][0] = pow(w_z, 2) + 2.0 * mu / pow(r, 3) + K * (1 - 3 * pow(sin(i), 2) * pow(sin(theta), 2));
  system_matrix[3][1] = w_z_dot + K * pow(sin(i), 2) * sin(2 * theta);
  system_matrix[3][2] = -w_x * w_z + K * sin(2 * i) * sin(theta);
  system_matrix[3][3] = 0;
  system_matrix[3][4] = - 2 * w_z;
  system_matrix[3][5] = 0;
  system_matrix[4][0] = -w_z_dot + K * sin(2 * i) * sin(theta);
  system_matrix[4][1] = pow(w_z, 2) + pow(w_x, 2) - mu / pow(r, 3) + K * (-1.0 / 4.0 + pow(sin(i), 2) * (7.0 / 4.0 * pow(sin(theta), 2) - 1.0 / 2.0));
  system_matrix[4][2] = w_x_dot + K * (-1.0 / 4.0 - sin(2 * i) * cos(theta));
  system_matrix[4][3] = - 2 * w_z;
  system_matrix[4][4] = 0;
  system_matrix[4][5] = 2 * w_x;
  system_matrix[5][0] = -w_x * w_z + K * sin(2 * i) * sin(theta);
  system_matrix[5][1] = -w_x_dot + K * (-1.0 / 4.0) * sin(2 * i) * cos(theta);
  system_matrix[5][2] = pow(w_x, 2) - mu / pow(r, 3)
                 + K * ( (-3.0 / 4.0) + pow(sin(i), 2)
                 * ( (5.0 / 4.0) * pow(sin(theta), 2) + 1.0 / 2.0) );
  system_matrix[5][3] = 0;
  system_matrix[5][4] = - 2 * w_x;
  system_matrix[5][5] = 0;

  return system_matrix;
}

}  // namespace orbit
