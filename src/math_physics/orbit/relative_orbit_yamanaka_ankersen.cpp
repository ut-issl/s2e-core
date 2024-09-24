/**
 * @file relative_orbit_yamanaka_ankersen.cpp
 * @brief Functions to calculate Yamanaka-Ankersen STM for relative orbit
 */
#include "relative_orbit_yamanaka_ankersen.hpp"

#include <environment/global/physical_constants.hpp>

#include "./relative_orbit_models.hpp"
#include "./sgp4/sgp4unit.h"  // for getgravconst()

namespace orbit {

RelativeOrbitYamanakaAnkersen::RelativeOrbitYamanakaAnkersen() {}

RelativeOrbitYamanakaAnkersen::~RelativeOrbitYamanakaAnkersen() {}

void RelativeOrbitYamanakaAnkersen::CalculateInitialInverseMatrix(double gravity_constant_m3_s2, double f_ref_rad, OrbitalElements* reference_oe) {
  // The following variables are defined in Spacecraft Formation Flying (Section 5.6.4)
  const double e = reference_oe->GetEccentricity();
  const double a = reference_oe->GetSemiMajorAxis_m();
  const double h = pow(a * (1.0 - pow(e, 2)) * gravity_constant_m3_s2, 0.5);  // angular momentum

  const double eta = pow(1 - pow(e, 2.0), 0.5);
  const double k = 1 + e * cos(f_ref_rad);
  const double c = k * cos(f_ref_rad);
  const double s = k * sin(f_ref_rad);

  initial_inverse_matrix_[0][0] = -3.0 * s * ((k + pow(e, 2.0)) / pow(k, 2.0));
  initial_inverse_matrix_[0][1] = 0.0;
  initial_inverse_matrix_[0][2] = 0.0;
  initial_inverse_matrix_[0][3] = c - 2.0 * e;
  initial_inverse_matrix_[0][4] = -s * (k + 1) / k;
  initial_inverse_matrix_[0][5] = 0.0;

  initial_inverse_matrix_[1][0] = 3.0 * k - pow(eta, 2.0);
  initial_inverse_matrix_[1][1] = 0.0;
  initial_inverse_matrix_[1][2] = 0.0;
  initial_inverse_matrix_[1][3] = e * s;
  initial_inverse_matrix_[1][4] = pow(k, 2.0);
  initial_inverse_matrix_[1][5] = 0.0;

  initial_inverse_matrix_[2][0] = 0.0;
  initial_inverse_matrix_[2][1] = 0.0;
  initial_inverse_matrix_[2][2] = pow(eta, 2.0) * cos(f_ref_rad);
  initial_inverse_matrix_[2][3] = 0.0;
  initial_inverse_matrix_[2][4] = 0.0;
  initial_inverse_matrix_[2][5] = -pow(eta, 2.0) * sin(f_ref_rad);

  initial_inverse_matrix_[3][0] = -3.0 * (e + c / k);
  initial_inverse_matrix_[3][1] = 0.0;
  initial_inverse_matrix_[3][2] = 0.0;
  initial_inverse_matrix_[3][3] = -s;
  initial_inverse_matrix_[3][4] = -(c * (k + 1) / k + e);
  initial_inverse_matrix_[3][5] = 0.0;

  initial_inverse_matrix_[4][0] = -3.0 * e * s * (k + 1) / pow(k, 2.0);
  initial_inverse_matrix_[4][1] = pow(eta, 2.0);
  initial_inverse_matrix_[4][2] = 0.0;
  initial_inverse_matrix_[4][3] = -2.0 + e * c;
  initial_inverse_matrix_[4][4] = -e * s * (k + 1) / k;
  initial_inverse_matrix_[4][5] = 0.0;

  initial_inverse_matrix_[5][0] = 0.0;
  initial_inverse_matrix_[5][1] = 0.0;
  initial_inverse_matrix_[5][2] = pow(eta, 2.0) * sin(f_ref_rad);
  initial_inverse_matrix_[5][3] = 0.0;
  initial_inverse_matrix_[5][4] = 0.0;
  initial_inverse_matrix_[5][5] = pow(eta, 2.0) * cos(f_ref_rad);

  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      initial_inverse_matrix_[i][j] = 1 / pow(eta, 2.0) * initial_inverse_matrix_[i][j];
    }
  }

  initial_inverse_matrix_ =
      initial_inverse_matrix_ * orbit::CalcStateTransformationMatrixLvlhToTschaunerHampel(gravity_constant_m3_s2, e, h, f_ref_rad);
}

math::Matrix<6, 6> RelativeOrbitYamanakaAnkersen::CalculateSTM(double gravity_constant_m3_s2, double elapsed_time_s, double f_ref_rad,
                                                               OrbitalElements* reference_oe) {
  // The following variables are defined in Spacecraft Formation Flying (Section 5.6.4)
  const double e = reference_oe->GetEccentricity();
  const double a = reference_oe->GetSemiMajorAxis_m();
  const double h = pow(a * (1 - pow(e, 2)) * gravity_constant_m3_s2, 0.5);  // angular momentum
  const double I = pow(gravity_constant_m3_s2, 2.0) / pow(h, 3.0);
  const double k = 1 + e * cos(f_ref_rad);
  const double c = k * cos(f_ref_rad);
  const double s = k * sin(f_ref_rad);
  const double c_prime = -sin(f_ref_rad) - e * sin(2 * f_ref_rad);
  const double s_prime = cos(f_ref_rad) + e * cos(2 * f_ref_rad);

  math::Matrix<6, 6> update_matrix;

  update_matrix[0][0] = s;
  update_matrix[0][1] = 2.0 - 3.0 * e * s * I;
  update_matrix[0][2] = 0.0;
  update_matrix[0][3] = c;
  update_matrix[0][4] = 0.0;
  update_matrix[0][5] = 0.0;

  update_matrix[1][0] = c * (1 + 1 / k);
  update_matrix[1][1] = -3.0 * pow(k, 2.0) * I;
  update_matrix[1][2] = 0.0;
  update_matrix[1][3] = -s * (1.0 + 1.0 / k);
  update_matrix[1][4] = 1.0;
  update_matrix[1][5] = 0.0;

  update_matrix[2][0] = 0.0;
  update_matrix[2][1] = 0.0;
  update_matrix[2][2] = cos(f_ref_rad);
  update_matrix[2][3] = 0.0;
  update_matrix[2][4] = 0.0;
  update_matrix[2][5] = sin(f_ref_rad);

  update_matrix[3][0] = s_prime;
  update_matrix[3][1] = -3.0 * e * (s_prime * I + s / pow(k, 2.0));
  update_matrix[3][2] = 0.0;
  update_matrix[3][3] = c_prime;
  update_matrix[3][4] = 0.0;
  update_matrix[3][5] = 0.0;

  update_matrix[4][0] = -2.0 * s;
  update_matrix[4][1] = -3.0 * (1.0 - 2.0 * e * s * I);
  update_matrix[4][2] = 0.0;
  update_matrix[4][3] = e - 2.0 * c;
  update_matrix[4][4] = 0.0;
  update_matrix[4][5] = 0.0;

  update_matrix[5][0] = 0.0;
  update_matrix[5][1] = 0.0;
  update_matrix[5][2] = -sin(f_ref_rad);
  update_matrix[5][3] = 0.0;
  update_matrix[5][4] = 0.0;
  update_matrix[5][5] = cos(f_ref_rad);

  return orbit::CalcStateTransformationMatrixTschaunerHampelToLvlh(gravity_constant_m3_s2, e, h, f_ref_rad) * update_matrix * initial_inverse_matrix_;
}

}  // namespace orbit
