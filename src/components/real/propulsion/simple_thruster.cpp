/*
 * @file simple_thruster.cpp
 * @brief Component emulation of simple thruster
 */
#include "simple_thruster.hpp"

#include <cfloat>
#include <library/math/constants.hpp>
#include <library/randomization/global_randomization.hpp>

// Constructor
SimpleThruster::SimpleThruster(const int prescaler, ClockGenerator* clock_generator, const int component_id, const Vector<3> thruster_pos_b,
                               const Vector<3> thrust_dir_b, const double max_mag, const double mag_err, const double dir_err,
                               const Structure* structure, const Dynamics* dynamics)
    : Component(prescaler, clock_generator),
      component_id_(component_id),
      thruster_position_b_m_(thruster_pos_b),
      thrust_direction_b_(thrust_dir_b),
      thrust_magnitude_max_N_(max_mag),
      direction_noise_standard_deviation_rad_(dir_err),
      structure_(structure),
      dynamics_(dynamics) {
  Initialize(mag_err, dir_err);
}

SimpleThruster::SimpleThruster(const int prescaler, ClockGenerator* clock_generator, PowerPort* power_port, const int component_id,
                               const Vector<3> thruster_pos_b, const Vector<3> thrust_dir_b, const double max_mag, const double mag_err,
                               const double dir_err, const Structure* structure, const Dynamics* dynamics)
    : Component(prescaler, clock_generator, power_port),
      component_id_(component_id),
      thruster_position_b_m_(thruster_pos_b),
      thrust_direction_b_(thrust_dir_b),
      thrust_magnitude_max_N_(max_mag),
      direction_noise_standard_deviation_rad_(dir_err),
      structure_(structure),
      dynamics_(dynamics) {
  Initialize(mag_err, dir_err);
}

SimpleThruster::~SimpleThruster() {}

void SimpleThruster::Initialize(const double mag_err, const double dir_err) {
  magnitude_random_noise_.SetParameters(0.0, mag_err);
  direction_random_noise_.SetParameters(0.0, dir_err);
  thrust_direction_b_ = Normalize(thrust_direction_b_);
}

void SimpleThruster::MainRoutine(int count) {
  UNUSED(count);

  CalcThrust();
  CalcTorque(structure_->GetKinematicsParams().GetCGb());
}

void SimpleThruster::PowerOffRoutine() {
  output_thrust_b_N_ *= 0.0;
  output_torque_b_Nm_ *= 0.0;
}

void SimpleThruster::CalcThrust() {
  double mag = CalcThrustMagnitude();
  if (duty_ > 0.0 + DBL_EPSILON) mag += magnitude_random_noise_;
  output_thrust_b_N_ = mag * CalcThrustDir();
}

void SimpleThruster::CalcTorque(Vector<3> center) {
  Vector<3> vector_center2thruster = thruster_position_b_m_ - center;
  Vector<3> torque = OuterProduct(vector_center2thruster, output_thrust_b_N_);

  output_torque_b_Nm_ = torque;
}

std::string SimpleThruster::GetLogHeader() const {
  std::string str_tmp = "";

  std::string head = "simple_thruster" + std::to_string(component_id_) + "_";
  str_tmp += WriteVector(head + "output_thrust", "b", "N", 3);
  str_tmp += WriteVector(head + "output_torque", "b", "Nm", 3);
  str_tmp += WriteScalar(head + "output_thrust_norm", "N");
  return str_tmp;
}

std::string SimpleThruster::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteVector(output_thrust_b_N_);
  str_tmp += WriteVector(output_torque_b_Nm_);
  str_tmp += WriteScalar(CalcNorm(output_thrust_b_N_));

  return str_tmp;
}

double SimpleThruster::CalcThrustMagnitude() { return duty_ * thrust_magnitude_max_N_; }

Vector<3> SimpleThruster::CalcThrustDir() {
  Vector<3> thrust_dir_b_true = thrust_direction_b_;
  if (direction_noise_standard_deviation_rad_ > 0.0 + DBL_EPSILON) {
    Vector<3> ex;  // Fixme: to use outer product to generate orthogonal vector
    ex[0] = 1.0;
    ex[1] = 0.0;
    ex[2] = 0.0;
    int flag = rand() % 2;
    double make_axis_rot_rad;
    if (flag == 0) {
      make_axis_rot_rad = libra::pi * (double)rand() / RAND_MAX;
    } else {
      make_axis_rot_rad = -libra::pi * (double)rand() / RAND_MAX;
    }

    Quaternion make_axis_rot(thrust_dir_b_true, make_axis_rot_rad);
    Vector<3> axis_rot = make_axis_rot.FrameConversion(ex);

    Quaternion err_rot(axis_rot, direction_random_noise_);           // Generate error quaternion
    thrust_dir_b_true = err_rot.FrameConversion(thrust_dir_b_true);  // Add error
  }

  return thrust_dir_b_true;
}
