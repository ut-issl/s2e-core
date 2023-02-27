/*
 * @file reaction_wheel_jitter.cpp
 * @brief Class to calculate RW high-frequency jitter effect
 */

#include "reaction_wheel_jitter.hpp"

#include <library/math/constants.hpp>
#include <random>

RWJitter::RWJitter(std::vector<std::vector<double>> radial_force_harmonics_coef, std::vector<std::vector<double>> radial_torque_harmonics_coef,
                   const double jitter_update_interval, const libra::Quaternion quaternion_b2c, const double structural_resonance_freq,
                   const double damping_factor, const double bandwidth, const bool considers_structural_resonance)
    : radial_force_harmonics_coef_(radial_force_harmonics_coef),
      radial_torque_harmonics_coef_(radial_torque_harmonics_coef),
      jitter_update_interval_(jitter_update_interval),
      q_b2c_(quaternion_b2c),
      structural_resonance_freq_(structural_resonance_freq),
      structural_resonance_angular_freq_(libra::tau * structural_resonance_freq),
      damping_factor_(damping_factor),
      bandwidth_(bandwidth),
      considers_structural_resonance_(considers_structural_resonance) {
  // Generate random number for initial rotation phase
  std::random_device seed_gen;
  std::default_random_engine engine(seed_gen());
  std::uniform_real_distribution<double> dist(0.0, libra::tau);
  // Initialize RW rotation phase
  for (size_t i = 0; i < radial_force_harmonics_coef_.size(); i++) {
    jitter_force_rot_phase_.push_back(dist(engine));
  }
  for (size_t i = 0; i < radial_torque_harmonics_coef_.size(); i++) {
    jitter_torque_rot_phase_.push_back(dist(engine));
  }
  // Calculate the coefficients of the difference equation when structural resonance is considered
  if (considers_structural_resonance_) {
    CalcCoef();
  }
}

RWJitter::~RWJitter() {}

void RWJitter::CalcJitter(double angular_velocity_rad) {
  // Clear jitter in component frame
  unfiltered_jitter_force_n_c_ *= 0.0;
  unfiltered_jitter_torque_n_c_ *= 0.0;

  // Calculate harmonics force
  for (size_t i = 0; i < jitter_force_rot_phase_.size(); i++) {
    // Advance the phase of RW rotation
    jitter_force_rot_phase_[i] += radial_force_harmonics_coef_[i][0] * angular_velocity_rad * jitter_update_interval_;
    // Add jitter force
    unfiltered_jitter_force_n_c_[0] +=
        radial_force_harmonics_coef_[i][1] * angular_velocity_rad * angular_velocity_rad * sin(jitter_force_rot_phase_[i]);
    unfiltered_jitter_force_n_c_[1] +=
        radial_force_harmonics_coef_[i][1] * angular_velocity_rad * angular_velocity_rad * cos(jitter_force_rot_phase_[i]);
    // jitter_force_c_[2] += 0.0;
  }

  // Calculate harmonics torque
  for (size_t i = 0; i < jitter_torque_rot_phase_.size(); i++) {
    // Advance the phase of RW rotation
    jitter_torque_rot_phase_[i] += radial_torque_harmonics_coef_[i][0] * angular_velocity_rad * jitter_update_interval_;
    // Add jitter torque
    unfiltered_jitter_torque_n_c_[0] +=
        radial_torque_harmonics_coef_[i][1] * angular_velocity_rad * angular_velocity_rad * sin(jitter_torque_rot_phase_[i]);
    unfiltered_jitter_torque_n_c_[1] +=
        radial_torque_harmonics_coef_[i][1] * angular_velocity_rad * angular_velocity_rad * cos(jitter_torque_rot_phase_[i]);
    // jitter_torque_c_[2] += 0.0;
  }

  // Add structural resonance
  if (considers_structural_resonance_) {
    AddStructuralResonance();
    jitter_force_b_ = q_b2c_.InverseFrameConversion(filtered_jitter_force_n_c_);
    jitter_torque_b_ = q_b2c_.InverseFrameConversion(filtered_jitter_torque_n_c_);
  } else {
    jitter_force_b_ = q_b2c_.InverseFrameConversion(unfiltered_jitter_force_n_c_);
    jitter_torque_b_ = q_b2c_.InverseFrameConversion(unfiltered_jitter_torque_n_c_);
  }
}

void RWJitter::AddStructuralResonance() {
  // Solve difference equations
  for (int i = 0; i < 3; i++) {
    filtered_jitter_force_n_c_[i] =
        (-coef_[1] * filtered_jitter_force_n_1_c_[i] - coef_[2] * filtered_jitter_force_n_2_c_[i] + coef_[3] * unfiltered_jitter_force_n_c_[i] +
         coef_[4] * unfiltered_jitter_force_n_1_c_[i] + coef_[5] * unfiltered_jitter_force_n_2_c_[i]) /
        coef_[0];

    filtered_jitter_torque_n_c_[i] =
        (-coef_[1] * filtered_jitter_torque_n_1_c_[i] - coef_[2] * filtered_jitter_torque_n_2_c_[i] + coef_[3] * unfiltered_jitter_torque_n_c_[i] +
         coef_[4] * unfiltered_jitter_torque_n_1_c_[i] + coef_[5] * unfiltered_jitter_torque_n_2_c_[i]) /
        coef_[0];
  }

  ShiftTimeStep();
}

void RWJitter::ShiftTimeStep() {
  unfiltered_jitter_force_n_2_c_ = unfiltered_jitter_force_n_1_c_;
  unfiltered_jitter_force_n_1_c_ = unfiltered_jitter_force_n_c_;
  filtered_jitter_force_n_2_c_ = filtered_jitter_force_n_1_c_;
  filtered_jitter_force_n_1_c_ = filtered_jitter_force_n_c_;

  unfiltered_jitter_torque_n_2_c_ = unfiltered_jitter_torque_n_1_c_;
  unfiltered_jitter_torque_n_1_c_ = unfiltered_jitter_torque_n_c_;
  filtered_jitter_torque_n_2_c_ = filtered_jitter_torque_n_1_c_;
  filtered_jitter_torque_n_1_c_ = filtered_jitter_torque_n_c_;
}

void RWJitter::CalcCoef() {
  // Pre-warping
  structural_resonance_angular_freq_ = 2.0 / jitter_update_interval_ * tan(structural_resonance_angular_freq_ * jitter_update_interval_ / 2.0);
  // Calculate coefficients of difference equation
  coef_[0] = 4.0 + 4.0 * bandwidth_ * damping_factor_ * jitter_update_interval_ * structural_resonance_angular_freq_ +
             pow(jitter_update_interval_, 2.0) * pow(structural_resonance_angular_freq_, 2.0);
  coef_[1] = -8.0 + 2.0 * pow(jitter_update_interval_, 2.0) * pow(structural_resonance_angular_freq_, 2.0);
  coef_[2] = 4.0 - 4.0 * bandwidth_ * damping_factor_ * jitter_update_interval_ * structural_resonance_angular_freq_ +
             pow(jitter_update_interval_, 2.0) * pow(structural_resonance_angular_freq_, 2.0);
  coef_[3] = 4.0 + 4.0 * damping_factor_ * jitter_update_interval_ * structural_resonance_angular_freq_ +
             pow(jitter_update_interval_, 2.0) * pow(structural_resonance_angular_freq_, 2.0);
  coef_[4] = -8.0 + 2.0 * pow(jitter_update_interval_, 2.0) * pow(structural_resonance_angular_freq_, 2.0);
  coef_[5] = 4.0 - 4.0 * damping_factor_ * jitter_update_interval_ * structural_resonance_angular_freq_ +
             pow(jitter_update_interval_, 2.0) * pow(structural_resonance_angular_freq_, 2.0);
}
