/*
 * @file force_generator.cpp
 * @brief Ideal component which can generate force for control algorithm test
 */

#include "force_generator.hpp"

#include <cfloat>
#include <setting_file_reader/initialize_file_access.hpp>

// Constructor
ForceGenerator::ForceGenerator(const int prescaler, ClockGenerator* clock_generator, const double magnitude_error_standard_deviation_N,
                               const double direction_error_standard_deviation_rad, const Dynamics* dynamics)
    : Component(prescaler, clock_generator),
      magnitude_noise_(0.0, magnitude_error_standard_deviation_N),
      direction_error_standard_deviation_rad_(direction_error_standard_deviation_rad),
      dynamics_(dynamics) {
  direction_noise_.SetParameters(0.0, 1.0);
}

ForceGenerator::~ForceGenerator() {}

void ForceGenerator::MainRoutine(const int time_count) {
  UNUSED(time_count);

  generated_force_b_N_ = ordered_force_b_N_;

  // Add noise
  double norm_ordered_force = ordered_force_b_N_.CalcNorm();
  if (norm_ordered_force > 0.0 + DBL_EPSILON) {
    // Add noise only when the force is generated
    math::Vector<3> true_direction = generated_force_b_N_.CalcNormalizedVector();
    libra::Quaternion error_quaternion = GenerateDirectionNoiseQuaternion(true_direction, direction_error_standard_deviation_rad_);
    math::Vector<3> converted_direction = error_quaternion.FrameConversion(true_direction);
    double force_norm_with_error = norm_ordered_force + magnitude_noise_;
    generated_force_b_N_ = force_norm_with_error * converted_direction;
  }

  // Convert frame
  libra::Quaternion q_i2b = dynamics_->GetAttitude().GetQuaternion_i2b();
  libra::Quaternion q_i2rtn = dynamics_->GetOrbit().CalcQuaternion_i2lvlh();
  generated_force_i_N_ = q_i2b.InverseFrameConversion(generated_force_b_N_);
  generated_force_rtn_N_ = q_i2rtn.FrameConversion(generated_force_i_N_);
}

void ForceGenerator::PowerOffRoutine() {
  generated_force_b_N_ *= 0.0;
  generated_force_i_N_ *= 0.0;
  generated_force_rtn_N_ *= 0.0;
}

void ForceGenerator::SetForce_i_N(const math::Vector<3> force_i_N) {
  libra::Quaternion q_i2b = dynamics_->GetAttitude().GetQuaternion_i2b();
  ordered_force_b_N_ = q_i2b.FrameConversion(force_i_N);
}

void ForceGenerator::SetForce_rtn_N(const math::Vector<3> force_rtn_N) {
  libra::Quaternion q_i2b = dynamics_->GetAttitude().GetQuaternion_i2b();
  libra::Quaternion q_i2rtn = dynamics_->GetOrbit().CalcQuaternion_i2lvlh();

  math::Vector<3> force_i_N = q_i2rtn.InverseFrameConversion(force_rtn_N);
  ordered_force_b_N_ = q_i2b.FrameConversion(force_i_N);
}

std::string ForceGenerator::GetLogHeader() const {
  std::string str_tmp = "";

  std::string head = "ideal_force_generator_";
  str_tmp += WriteVector(head + "ordered_force", "b", "N", 3);
  str_tmp += WriteVector(head + "generated_force", "b", "N", 3);
  str_tmp += WriteVector(head + "generated_force", "i", "N", 3);
  str_tmp += WriteVector(head + "generated_force", "rtn", "N", 3);

  return str_tmp;
}

std::string ForceGenerator::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteVector(ordered_force_b_N_);
  str_tmp += WriteVector(generated_force_b_N_);
  str_tmp += WriteVector(generated_force_i_N_);
  str_tmp += WriteVector(generated_force_rtn_N_);

  return str_tmp;
}

libra::Quaternion ForceGenerator::GenerateDirectionNoiseQuaternion(math::Vector<3> true_direction, const double error_standard_deviation_rad) {
  math::Vector<3> random_direction;
  random_direction[0] = direction_noise_;
  random_direction[1] = direction_noise_;
  random_direction[2] = direction_noise_;
  random_direction = random_direction.CalcNormalizedVector();

  math::Vector<3> rotation_axis;
  rotation_axis = OuterProduct(true_direction, random_direction);
  double norm_rotation_axis = rotation_axis.CalcNorm();
  if (norm_rotation_axis < 0.0 + DBL_EPSILON) {
    // No rotation error if the randomized direction is parallel to the true direction
    rotation_axis = true_direction;
  }

  double error_angle_rad = direction_noise_ * error_standard_deviation_rad;
  libra::Quaternion error_quaternion(rotation_axis, error_angle_rad);
  return error_quaternion;
}

ForceGenerator InitializeForceGenerator(ClockGenerator* clock_generator, const std::string file_name, const Dynamics* dynamics) {
  // General
  IniAccess ini_file(file_name);

  // CompoBase
  int prescaler = ini_file.ReadInt("COMPONENT_BASE", "prescaler");
  if (prescaler <= 1) prescaler = 1;

  // ForceGenerator
  char section[30] = "FORCE_GENERATOR";
  double force_magnitude_standard_deviation_N = ini_file.ReadDouble(section, "force_magnitude_standard_deviation_N");
  double force_direction_standard_deviation_deg = ini_file.ReadDouble(section, "force_direction_standard_deviation_deg");
  double force_direction_standard_deviation_rad = libra::deg_to_rad * force_direction_standard_deviation_deg;
  ForceGenerator force_generator(prescaler, clock_generator, force_magnitude_standard_deviation_N, force_direction_standard_deviation_rad, dynamics);

  return force_generator;
}
