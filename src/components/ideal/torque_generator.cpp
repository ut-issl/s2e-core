/*
 * @file torque_generator.cpp
 * @brief Ideal component which can generate torque for control algorithm test
 */

#include "torque_generator.hpp"

#include <cfloat>
#include <setting_file_reader/initialize_file_access.hpp>

// Constructor
TorqueGenerator::TorqueGenerator(const int prescaler, ClockGenerator* clock_generator, const double magnitude_error_standard_deviation_Nm,
                                 const double direction_error_standard_deviation_rad, const Dynamics* dynamics)
    : Component(prescaler, clock_generator),
      magnitude_noise_(0.0, magnitude_error_standard_deviation_Nm),
      direction_error_standard_deviation_rad_(direction_error_standard_deviation_rad),
      dynamics_(dynamics) {
  direction_noise_.SetParameters(0.0, 1.0);
}

TorqueGenerator::~TorqueGenerator() {}

void TorqueGenerator::MainRoutine(const int time_count) {
  UNUSED(time_count);

  generated_torque_b_Nm_ = ordered_torque_b_Nm_;

  // Add noise
  double norm_ordered_torque = ordered_torque_b_Nm_.CalcNorm();
  if (norm_ordered_torque > 0.0 + DBL_EPSILON) {
    // Add noise only when the torque is generated
    math::Vector<3> true_direction = generated_torque_b_Nm_.CalcNormalizedVector();
    math::Quaternion error_quaternion = GenerateDirectionNoiseQuaternion(true_direction, direction_error_standard_deviation_rad_);
    math::Vector<3> converted_direction = error_quaternion.FrameConversion(true_direction);
    double torque_norm_with_error = norm_ordered_torque + magnitude_noise_;
    generated_torque_b_Nm_ = torque_norm_with_error * converted_direction;
  }
}

void TorqueGenerator::PowerOffRoutine() { generated_torque_b_Nm_ *= 0.0; }

std::string TorqueGenerator::GetLogHeader() const {
  std::string str_tmp = "";

  std::string head = "ideal_torque_generator_";
  str_tmp += WriteVector(head + "ordered_torque", "b", "Nm", 3);
  str_tmp += WriteVector(head + "generated_torque", "b", "Nm", 3);

  return str_tmp;
}

std::string TorqueGenerator::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteVector(ordered_torque_b_Nm_);
  str_tmp += WriteVector(generated_torque_b_Nm_);

  return str_tmp;
}

math::Quaternion TorqueGenerator::GenerateDirectionNoiseQuaternion(math::Vector<3> true_direction, const double error_standard_deviation_rad) {
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
  math::Quaternion error_quaternion(rotation_axis, error_angle_rad);
  return error_quaternion;
}

TorqueGenerator InitializeTorqueGenerator(ClockGenerator* clock_generator, const std::string file_name, const Dynamics* dynamics) {
  // General
  IniAccess ini_file(file_name);

  // CompoBase
  int prescaler = ini_file.ReadInt("COMPONENT_BASE", "prescaler");
  if (prescaler <= 1) prescaler = 1;

  // TorqueGenerator
  char section[30] = "TORQUE_GENERATOR";
  double torque_magnitude_standard_deviation_Nm = ini_file.ReadDouble(section, "torque_magnitude_standard_deviation_Nm");
  double torque_direction_standard_deviation_deg = ini_file.ReadDouble(section, "torque_direction_standard_deviation_deg");
  double torque_direction_standard_deviation_rad = math::deg_to_rad * torque_direction_standard_deviation_deg;
  TorqueGenerator torque_generator(prescaler, clock_generator, torque_magnitude_standard_deviation_Nm, torque_direction_standard_deviation_rad,
                                   dynamics);

  return torque_generator;
}
