/*
 * @file reaction_wheel_jitter.cpp
 * @brief Class to calculate RW high-frequency jitter effect
 */

#include "reaction_wheel_jitter.hpp"

#include <math_physics/math/constants.hpp>
#include <random>
#include <utilities/macros.hpp>

namespace s2e::components {

ReactionWheelJitter::ReactionWheelJitter(std::vector<std::vector<double>> radial_force_harmonics_coefficients,
                                         std::vector<std::vector<double>> radial_torque_harmonics_coefficients, const double update_interval_s,
                                         const math::Quaternion quaternion_b2c, const double structural_resonance_frequency_Hz,
                                         const double damping_factor, const double bandwidth, const bool considers_structural_resonance)
    : radial_force_harmonics_coefficients_(radial_force_harmonics_coefficients),
      radial_torque_harmonics_coefficients_(radial_torque_harmonics_coefficients),
      update_interval_s_(update_interval_s),
      quaternion_b2c_(quaternion_b2c),
      structural_resonance_frequency_Hz_(structural_resonance_frequency_Hz),
      structural_resonance_angular_frequency_Hz_(math::tau * structural_resonance_frequency_Hz),
      damping_factor_(damping_factor),
      bandwidth_(bandwidth),
      considers_structural_resonance_(considers_structural_resonance) {
  UNUSED(structural_resonance_frequency_Hz_);  // TODO: implement algorithm to use this variable
  // Generate random number for initial rotation phase
  std::random_device seed_gen;
  std::default_random_engine engine(seed_gen());
  std::uniform_real_distribution<double> dist(0.0, math::tau);
  // Initialize RW rotation phase
  for (size_t i = 0; i < radial_force_harmonics_coefficients_.size(); i++) {
    jitter_force_rotation_phase_.push_back(dist(engine));
  }
  for (size_t i = 0; i < radial_torque_harmonics_coefficients_.size(); i++) {
    jitter_torque_rotation_phase_.push_back(dist(engine));
  }
  // Calculate the coefficients of the difference equation when structural resonance is considered
  if (considers_structural_resonance_) {
    CalcCoefficients();
  }
}

ReactionWheelJitter::~ReactionWheelJitter() {}

void ReactionWheelJitter::CalcJitter(double angular_velocity_rad) {
  // Clear jitter in component frame
  unfiltered_jitter_force_n_c_ *= 0.0;
  unfiltered_jitter_torque_n_c_ *= 0.0;

  // Calculate harmonics force
  for (size_t i = 0; i < jitter_force_rotation_phase_.size(); i++) {
    // Advance the phase of RW rotation
    jitter_force_rotation_phase_[i] += radial_force_harmonics_coefficients_[i][0] * angular_velocity_rad * update_interval_s_;
    // Add jitter force
    unfiltered_jitter_force_n_c_[0] +=
        radial_force_harmonics_coefficients_[i][1] * angular_velocity_rad * angular_velocity_rad * sin(jitter_force_rotation_phase_[i]);
    unfiltered_jitter_force_n_c_[1] +=
        radial_force_harmonics_coefficients_[i][1] * angular_velocity_rad * angular_velocity_rad * cos(jitter_force_rotation_phase_[i]);
    // jitter_force_c_[2] += 0.0;
  }

  // Calculate harmonics torque
  for (size_t i = 0; i < jitter_torque_rotation_phase_.size(); i++) {
    // Advance the phase of RW rotation
    jitter_torque_rotation_phase_[i] += radial_torque_harmonics_coefficients_[i][0] * angular_velocity_rad * update_interval_s_;
    // Add jitter torque
    unfiltered_jitter_torque_n_c_[0] +=
        radial_torque_harmonics_coefficients_[i][1] * angular_velocity_rad * angular_velocity_rad * sin(jitter_torque_rotation_phase_[i]);
    unfiltered_jitter_torque_n_c_[1] +=
        radial_torque_harmonics_coefficients_[i][1] * angular_velocity_rad * angular_velocity_rad * cos(jitter_torque_rotation_phase_[i]);
    // jitter_torque_c_[2] += 0.0;
  }

  // Add structural resonance
  if (considers_structural_resonance_) {
    AddStructuralResonance();
    jitter_force_b_N_ = quaternion_b2c_.InverseFrameConversion(filtered_jitter_force_n_c_);
    jitter_torque_b_Nm_ = quaternion_b2c_.InverseFrameConversion(filtered_jitter_torque_n_c_);
  } else {
    jitter_force_b_N_ = quaternion_b2c_.InverseFrameConversion(unfiltered_jitter_force_n_c_);
    jitter_torque_b_Nm_ = quaternion_b2c_.InverseFrameConversion(unfiltered_jitter_torque_n_c_);
  }
}

void ReactionWheelJitter::AddStructuralResonance() {
  // Solve difference equations
  for (int i = 0; i < 3; i++) {
    filtered_jitter_force_n_c_[i] = (-coefficients_[1] * filtered_jitter_force_n_1_c_[i] - coefficients_[2] * filtered_jitter_force_n_2_c_[i] +
                                     coefficients_[3] * unfiltered_jitter_force_n_c_[i] + coefficients_[4] * unfiltered_jitter_force_n_1_c_[i] +
                                     coefficients_[5] * unfiltered_jitter_force_n_2_c_[i]) /
                                    coefficients_[0];

    filtered_jitter_torque_n_c_[i] = (-coefficients_[1] * filtered_jitter_torque_n_1_c_[i] - coefficients_[2] * filtered_jitter_torque_n_2_c_[i] +
                                      coefficients_[3] * unfiltered_jitter_torque_n_c_[i] + coefficients_[4] * unfiltered_jitter_torque_n_1_c_[i] +
                                      coefficients_[5] * unfiltered_jitter_torque_n_2_c_[i]) /
                                     coefficients_[0];
  }

  ShiftTimeStep();
}

void ReactionWheelJitter::ShiftTimeStep() {
  unfiltered_jitter_force_n_2_c_ = unfiltered_jitter_force_n_1_c_;
  unfiltered_jitter_force_n_1_c_ = unfiltered_jitter_force_n_c_;
  filtered_jitter_force_n_2_c_ = filtered_jitter_force_n_1_c_;
  filtered_jitter_force_n_1_c_ = filtered_jitter_force_n_c_;

  unfiltered_jitter_torque_n_2_c_ = unfiltered_jitter_torque_n_1_c_;
  unfiltered_jitter_torque_n_1_c_ = unfiltered_jitter_torque_n_c_;
  filtered_jitter_torque_n_2_c_ = filtered_jitter_torque_n_1_c_;
  filtered_jitter_torque_n_1_c_ = filtered_jitter_torque_n_c_;
}

void ReactionWheelJitter::CalcCoefficients() {
  // Pre-warping
  structural_resonance_angular_frequency_Hz_ = 2.0 / update_interval_s_ * tan(structural_resonance_angular_frequency_Hz_ * update_interval_s_ / 2.0);
  // Calculate coefficients of difference equation
  coefficients_[0] = 4.0 + 4.0 * bandwidth_ * damping_factor_ * update_interval_s_ * structural_resonance_angular_frequency_Hz_ +
                     pow(update_interval_s_, 2.0) * pow(structural_resonance_angular_frequency_Hz_, 2.0);
  coefficients_[1] = -8.0 + 2.0 * pow(update_interval_s_, 2.0) * pow(structural_resonance_angular_frequency_Hz_, 2.0);
  coefficients_[2] = 4.0 - 4.0 * bandwidth_ * damping_factor_ * update_interval_s_ * structural_resonance_angular_frequency_Hz_ +
                     pow(update_interval_s_, 2.0) * pow(structural_resonance_angular_frequency_Hz_, 2.0);
  coefficients_[3] = 4.0 + 4.0 * damping_factor_ * update_interval_s_ * structural_resonance_angular_frequency_Hz_ +
                     pow(update_interval_s_, 2.0) * pow(structural_resonance_angular_frequency_Hz_, 2.0);
  coefficients_[4] = -8.0 + 2.0 * pow(update_interval_s_, 2.0) * pow(structural_resonance_angular_frequency_Hz_, 2.0);
  coefficients_[5] = 4.0 - 4.0 * damping_factor_ * update_interval_s_ * structural_resonance_angular_frequency_Hz_ +
                     pow(update_interval_s_, 2.0) * pow(structural_resonance_angular_frequency_Hz_, 2.0);
}

}  // namespace s2e::components
