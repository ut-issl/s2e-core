/**
 * @file relative_orbit_carter.cpp
 * @brief Functions to calculate Carter's STM for relative orbit
 */
#include "relative_orbit_carter.hpp"

#include <environment/global/physical_constants.hpp>

#include "./relative_orbit_models.hpp"
#include "./sgp4/sgp4unit.h"  // for getgravconst()

namespace orbit {

RelativeOrbitCarter::RelativeOrbitCarter() {}

RelativeOrbitCarter::~RelativeOrbitCarter() {}

void RelativeOrbitCarter::CalculateInitialInverseMatrix(double gravity_constant_m3_s2, double f_ref_rad, OrbitalElements* reference_oe) {
  double e = reference_oe->GetEccentricity();
  double a = reference_oe->GetSemiMajorAxis_m();
  double h = pow(a * (1.0 - pow(e, 2)) * gravity_constant_m3_s2, 0.5);  // angular momentum

  double E_rad = 2.0 * atan(sqrt((1.0 - e) / (1.0 + e)) * tan(f_ref_rad / 2.0));
  double k = e * cos(f_ref_rad) + 1.0;
  // double K1 = pow(1 - e * e, -2.5) * (-1.5 * e * E_rad + (1 + e * e) * sin(E_rad) - e * sin(2. * E_rad) / 4.);
  double K2 = pow(1.0 - pow(e, 2.0), -2.5) * (0.5 * E_rad - 0.25 * sin(2.0 * E_rad) - e * pow(sin(E_rad), 3.0) / 3.0);
  double phi1 = sin(f_ref_rad) * k;
  double phi2 = 2.0 * e * phi1 * (sin(f_ref_rad) / pow(k, 3) - 3.0 * e * K2) - cos(f_ref_rad) / k;
  double phi3 = 6.0 * e * phi1 * K2 - 2.0 * pow(sin(f_ref_rad), 2.0) / pow(k, 2.0) - pow(cos(f_ref_rad), 2.0) / k - pow(cos(f_ref_rad), 2.0);
  double phi1_prime = cos(f_ref_rad) * k - e * pow(sin(f_ref_rad), 2.0);
  double phi2_prime = -6.0 * pow(e, 2.0) * phi1_prime * K2 +
                      2.0 * e * sin(f_ref_rad) * (2.0 * cos(f_ref_rad) - 3.0 * e * pow(sin(f_ref_rad), 2.0) + 2.0 * e) / pow(k, 3.0) +
                      sin(f_ref_rad) / pow(k, 2.0);
  double phi3_prime = 6.0 * e * phi1_prime * K2 - (6.0 * e * pow(sin(f_ref_rad), 3.0) - 4.0 * sin(f_ref_rad) * (e + cos(f_ref_rad))) / pow(k, 3.0) +
                      0.5 * sin(2 * f_ref_rad) * (2.0 + (2.0 + e * cos(f_ref_rad)) * pow(k, 2.0));
  double S1 = -cos(f_ref_rad) * (1.0 + 0.5 * e * cos(f_ref_rad));
  double S2 = 3.0 * e * pow(k, 2.0) * K2 - sin(f_ref_rad) / k;
  double S3 = -6.0 * pow(k, 2.0) * K2 - (2.0 + k) / 2.0 / k * sin(2.0 * f_ref_rad);

  initial_inverse_matrix_[0][0] = 4.0 * S2 + phi2_prime;
  initial_inverse_matrix_[0][1] = 0.0;
  initial_inverse_matrix_[0][2] = 0.0;
  initial_inverse_matrix_[0][3] = -phi2;
  initial_inverse_matrix_[0][4] = 2.0 * S2;
  initial_inverse_matrix_[0][5] = 0.0;
  initial_inverse_matrix_[1][0] = -2.0;
  initial_inverse_matrix_[1][1] = 0.0;
  initial_inverse_matrix_[1][2] = 0.0;
  initial_inverse_matrix_[1][3] = 0.0;
  initial_inverse_matrix_[1][4] = -1.0;
  initial_inverse_matrix_[1][5] = 0.0;
  initial_inverse_matrix_[2][0] = 0.0;
  initial_inverse_matrix_[2][1] = 0.0;
  initial_inverse_matrix_[2][2] = cos(f_ref_rad);
  initial_inverse_matrix_[2][3] = 0.0;
  initial_inverse_matrix_[2][4] = 0.0;
  initial_inverse_matrix_[2][5] = -sin(f_ref_rad);
  initial_inverse_matrix_[3][0] = -(4.0 * S1 + phi1_prime);
  initial_inverse_matrix_[3][1] = 0.0;
  initial_inverse_matrix_[3][2] = 0.0;
  initial_inverse_matrix_[3][3] = phi1;
  initial_inverse_matrix_[3][4] = -2.0 * S1;
  initial_inverse_matrix_[3][5] = 0.0;
  initial_inverse_matrix_[4][0] = 2.0 * S3 + phi3_prime;
  initial_inverse_matrix_[4][1] = -1.0;
  initial_inverse_matrix_[4][2] = 0.0;
  initial_inverse_matrix_[4][3] = -phi3;
  initial_inverse_matrix_[4][4] = S3;
  initial_inverse_matrix_[4][5] = 0.0;
  initial_inverse_matrix_[5][0] = 0.0;
  initial_inverse_matrix_[5][1] = 0.0;
  initial_inverse_matrix_[5][2] = sin(f_ref_rad);
  initial_inverse_matrix_[5][3] = 0.0;
  initial_inverse_matrix_[5][4] = 0.0;
  initial_inverse_matrix_[5][5] = cos(f_ref_rad);

  initial_inverse_matrix_ =
      initial_inverse_matrix_ * orbit::CalcStateTransformationMatrixLvlhToTschaunerHampel(gravity_constant_m3_s2, e, h, f_ref_rad);
}

math::Matrix<6, 6> RelativeOrbitCarter::CalculateSTM(double orbit_radius_m, double gravity_constant_m3_s2, double f_ref_rad,
                                                     OrbitalElements* reference_oe) {
  math::Matrix<6, 6> stm;
  double e = reference_oe->GetEccentricity();
  double a = reference_oe->GetSemiMajorAxis_m();
  double h = pow(a * (1 - pow(e, 2)) * gravity_constant_m3_s2, 0.5);  // angular momentum
  double E_rad = 2.0 * atan(sqrt((1.0 - e) / (1.0 + e)) * tan(f_ref_rad / 2.0));
  double k = e * cos(f_ref_rad) + 1.0;
  // double K1 = pow(1 - e * e, -2.5) * (-1.5 * e * E_rad + (1 + e * e) * sin(E_rad) - e * sin(2. * E_rad) / 4.);
  double K2 = pow(1.0 - pow(e, 2.0), -2.5) * (0.5 * E_rad - 0.25 * sin(2.0 * E_rad) - e * pow(sin(E_rad), 3.0) / 3.0);
  double phi1 = sin(f_ref_rad) * k;
  double phi2 = 2.0 * e * phi1 * (sin(f_ref_rad) / pow(k, 3) - 3.0 * e * K2) - cos(f_ref_rad) / k;
  double phi3 = 6.0 * e * phi1 * K2 - 2.0 * pow(sin(f_ref_rad), 2.0) / pow(k, 2.0) - pow(cos(f_ref_rad), 2.0) / k - pow(cos(f_ref_rad), 2.0);
  double phi1_prime = cos(f_ref_rad) * k - e * pow(sin(f_ref_rad), 2.0);
  double phi2_prime = -6.0 * pow(e, 2.0) * phi1_prime * K2 +
                      2.0 * e * sin(f_ref_rad) * (2.0 * cos(f_ref_rad) - 3.0 * e * pow(sin(f_ref_rad), 2.0) + 2.0 * e) / pow(k, 3.0) +
                      sin(f_ref_rad) / pow(k, 2.0);
  double phi3_prime = 6.0 * e * phi1_prime * K2 - (6.0 * e * pow(sin(f_ref_rad), 3.0) - 4.0 * sin(f_ref_rad) * (e + cos(f_ref_rad))) / pow(k, 3.0) +
                      0.5 * sin(2 * f_ref_rad) * (2.0 + (2.0 + e * cos(f_ref_rad)) * pow(k, 2.0));
  double S1 = -cos(f_ref_rad) * (1.0 + 0.5 * e * cos(f_ref_rad));
  double S2 = 3.0 * e * pow(k, 2.0) * K2 - sin(f_ref_rad) / k;
  double S3 = -6.0 * pow(k, 2.0) * K2 - (2.0 + k) / 2.0 / k * sin(2.0 * f_ref_rad);

  stm[0][0] = phi1;
  stm[0][1] = phi3;
  stm[0][2] = 0.0;
  stm[0][3] = phi2;
  stm[0][4] = 0.0;
  stm[0][5] = 0.0;
  stm[1][0] = -2 * S1;
  stm[1][1] = -S3;
  stm[1][2] = 0.0;
  stm[1][3] = -2 * S2;
  stm[1][4] = -1;
  stm[1][5] = 0.0;
  stm[2][0] = 0.0;
  stm[2][1] = 0.0;
  stm[2][2] = cos(f_ref_rad);
  stm[2][3] = 0.0;
  stm[2][4] = 0.0;
  stm[2][5] = sin(f_ref_rad);
  stm[3][0] = phi1_prime;
  stm[3][1] = phi3_prime;
  stm[3][2] = 0.0;
  stm[3][3] = phi2_prime;
  stm[3][4] = 0.0;
  stm[3][5] = 0.0;
  stm[4][0] = -2 * phi1;
  stm[4][1] = -(2 * phi3 + 1);
  stm[4][2] = 0.0;
  stm[4][3] = -2 * phi2;
  stm[4][4] = 0.0;
  stm[4][5] = 0.0;
  stm[5][0] = 0.0;
  stm[5][1] = 0.0;
  stm[5][2] = -sin(f_ref_rad);
  stm[5][3] = 0.0;
  stm[5][4] = 0.0;
  stm[5][5] = cos(f_ref_rad);
  return orbit::CalcStateTransformationMatrixTschaunerHampelToLvlh(gravity_constant_m3_s2, e, h, f_ref_rad) * stm * initial_inverse_matrix_;
}

}  // namespace orbit
