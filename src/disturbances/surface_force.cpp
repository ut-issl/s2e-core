/**
 * @file surface_force.cpp
 * @brief Base class for disturbances acting on a spacecraft surface (e.g., SRP, Air drag, etc)
 */

#include "surface_force.hpp"

#include "../library/math/vector.hpp"

SurfaceForce::SurfaceForce(const vector<Surface>& surfaces, const libra::Vector<3>& center_of_gravity_b_m, const bool is_calculation_enabled)
    : SimpleDisturbance(is_calculation_enabled), surfaces_(surfaces), center_of_gravity_b_m_(center_of_gravity_b_m) {
  // Initialize vectors
  int num = surfaces_.size();
  normal_coefficients_.assign(num, 0.0);
  tangential_coefficients_.assign(num, 0.0);
  cos_theta_.assign(num, 0.0);
  sin_theta_.assign(num, 0.0);
}

libra::Vector<3> SurfaceForce::CalcTorqueForce(libra::Vector<3>& input_b, double item) {
  CalcTheta(input_b);
  CalcCoefficients(input_b, item);

  libra::Vector<3> force_b_N(0.0);
  libra::Vector<3> torque_b_Nm(0.0);
  libra::Vector<3> input_b_normal(input_b);
  normalize(input_b_normal);

  for (size_t i = 0; i < surfaces_.size(); i++) {
    if (cos_theta_[i] > 0.0) {  // if the surface faces to the disturbance source (sun or air)
      // calc direction of in-plane force
      libra::Vector<3> normal = surfaces_[i].GetNormal();
      libra::Vector<3> ncu = outer_product(input_b_normal, normal);
      libra::Vector<3> ncu_normalized = normalize(ncu);
      libra::Vector<3> in_plane_force_direction = outer_product(ncu_normalized, normal);
      // calc force
      libra::Vector<3> force_per_surface_b_N = -1.0 * normal_coefficients_[i] * normal + tangential_coefficients_[i] * in_plane_force_direction;
      force_b_N += force_per_surface_b_N;
      // calc torque
      torque_b_Nm += outer_product(surfaces_[i].GetPosition() - center_of_gravity_b_m_, force_per_surface_b_N);
    }
  }
  force_b_N_ = force_b_N;
  torque_b_Nm_ = torque_b_Nm;
  return torque_b_Nm_;
}

void SurfaceForce::CalcTheta(libra::Vector<3>& input_b) {
  libra::Vector<3> input_b_normal(input_b);
  normalize(input_b_normal);

  for (size_t i = 0; i < surfaces_.size(); i++) {
    cos_theta_[i] = inner_product(surfaces_[i].GetNormal(), input_b_normal);
    sin_theta_[i] = sqrt(1.0 - cos_theta_[i] * cos_theta_[i]);
  }
}
