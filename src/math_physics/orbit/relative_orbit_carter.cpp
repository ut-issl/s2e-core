/**
 * @file relative_orbit_models.cpp
 * @brief Functions to calculate Yamanaka-ANkersen STM for relative orbit
 */
#include "relative_orbit_carter.hpp"

#include <environment/global/physical_constants.hpp>

#include "./sgp4/sgp4unit.h"  // for getgravconst()

namespace orbit {

RelativeOrbitCarter::RelativeOrbitCarter() {}

RelativeOrbitCarter::~RelativeOrbitCarter() {}

void RelativeOrbitCarter::CalculateInitialInverseMatrix(double orbit_radius_m, double gravity_constant_m3_s2, double f_ref_rad,
                                                                  OrbitalElements* reference_oe) {
  double eccentricity = reference_oe->GetEccentricity();
  double eta = pow(1 + pow(eccentricity, 2.0), 0.5);
  double k = 1 + eccentricity * cos(f_ref_rad);
  double c = k * cos(f_ref_rad);
  double s = k * sin(f_ref_rad);
  initial_inverse_matrix_[0][0] = -3.0 * s * (k + pow(eccentricity, 2.0) / pow(k, 2.0));
  initial_inverse_matrix_[0][1] = c - 2.0 * eccentricity;
  initial_inverse_matrix_[0][2] = 0.0;
  initial_inverse_matrix_[0][3] = -s * (k + 1) / k;
  initial_inverse_matrix_[0][4] = 0.0;
  initial_inverse_matrix_[0][5] = 0.0;
  initial_inverse_matrix_[1][0] = -3.0 * (eccentricity + c / k);
  initial_inverse_matrix_[1][1] = -s;
  initial_inverse_matrix_[1][2] = 0.0;
  initial_inverse_matrix_[1][3] = -(c * (k + 1) / k + eccentricity);
  initial_inverse_matrix_[1][4] = 0.0;
  initial_inverse_matrix_[1][5] = 0.0;
  initial_inverse_matrix_[2][0] = 3.0 * k - pow(eta, 2.0);
  initial_inverse_matrix_[2][1] = eccentricity * s;
  initial_inverse_matrix_[2][2] = 0.0;
  initial_inverse_matrix_[2][3] = pow(k, 2.0);
  initial_inverse_matrix_[2][4] = 0.0;
  initial_inverse_matrix_[2][5] = 0.0;
  initial_inverse_matrix_[3][0] = 3.0 * eccentricity * s * (k + 1) / pow(k, 2.0);
  initial_inverse_matrix_[3][1] = -2.0 + eccentricity * c;
  initial_inverse_matrix_[3][2] = pow(eta, 2.0);
  initial_inverse_matrix_[3][3] = -eccentricity * s * (k + 1) / k;
  initial_inverse_matrix_[3][4] = 0.0;
  initial_inverse_matrix_[3][5] = 0.0;
  initial_inverse_matrix_[4][0] = 0.0;
  initial_inverse_matrix_[4][1] = 0.0;
  initial_inverse_matrix_[4][2] = 0.0;
  initial_inverse_matrix_[4][3] = 0.0;
  initial_inverse_matrix_[4][4] = 0.0;
  initial_inverse_matrix_[4][5] = 0.0;

  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      initial_inverse_matrix_[i][j] = 1 / pow(eta, 2.0) * initial_inverse_matrix_[0][0];
    }
  }
}

}  // namespace orbit
