/**
 * @file surface_force.cpp
 * @brief Base class for disturbances acting on a spacecraft surface (e.g., SRP, Air drag, etc)
 */

#include "surface_force.hpp"

#include "../math_physics/math/vector.hpp"

SurfaceForce::SurfaceForce(const std::vector<Surface>& surfaces, const s2e::math::Vector<3>& center_of_gravity_b_m, const bool is_calculation_enabled)
    : Disturbance(is_calculation_enabled, true), surfaces_(surfaces), center_of_gravity_b_m_(center_of_gravity_b_m) {
  // Initialize vectors
  size_t num = surfaces_.size();
  normal_coefficients_.assign(num, 0.0);
  tangential_coefficients_.assign(num, 0.0);
  cos_theta_.assign(num, 0.0);
  sin_theta_.assign(num, 0.0);
}

s2e::math::Vector<3> SurfaceForce::CalcTorqueForce(s2e::math::Vector<3>& input_direction_b, double item) {
  CalcTheta(input_direction_b);
  CalcCoefficients(input_direction_b, item);

  s2e::math::Vector<3> force_b_N(0.0);
  s2e::math::Vector<3> torque_b_Nm(0.0);
  s2e::math::Vector<3> input_b_normal = input_direction_b.CalcNormalizedVector();

  for (size_t i = 0; i < surfaces_.size(); i++) {
    if (cos_theta_[i] > 0.0) {  // if the surface faces to the disturbance source (sun or air)
      // calc direction of in-plane force
      s2e::math::Vector<3> normal = surfaces_[i].GetNormal_b();
      s2e::math::Vector<3> ncu = OuterProduct(input_b_normal, normal);
      s2e::math::Vector<3> ncu_normalized = ncu.CalcNormalizedVector();
      s2e::math::Vector<3> in_plane_force_direction = OuterProduct(ncu_normalized, normal);
      // calc force
      s2e::math::Vector<3> force_per_surface_b_N = -1.0 * normal_coefficients_[i] * normal + tangential_coefficients_[i] * in_plane_force_direction;
      force_b_N += force_per_surface_b_N;
      // calc torque
      torque_b_Nm += OuterProduct(surfaces_[i].GetPosition_b_m() - center_of_gravity_b_m_, force_per_surface_b_N);
    }
  }
  force_b_N_ = force_b_N;
  torque_b_Nm_ = torque_b_Nm;
  return torque_b_Nm_;
}

void SurfaceForce::CalcTheta(s2e::math::Vector<3>& input_direction_b) {
  s2e::math::Vector<3> input_b_normal = input_direction_b.CalcNormalizedVector();

  for (size_t i = 0; i < surfaces_.size(); i++) {
    cos_theta_[i] = InnerProduct(surfaces_[i].GetNormal_b(), input_b_normal);
    sin_theta_[i] = sqrt(1.0 - cos_theta_[i] * cos_theta_[i]);
  }
}
