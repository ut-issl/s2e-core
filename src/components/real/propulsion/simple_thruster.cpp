/*
 * @file simple_thruster.cpp
 * @brief Component emulation of simple thruster
 */
#include "simple_thruster.hpp"

#include <cfloat>
#include <library/math/constants.hpp>
#include <library/randomization/global_randomization.hpp>

// Constructor
SimpleThruster::SimpleThruster(const int prescaler, ClockGenerator* clock_generator, const int id, const Vector<3> thruster_pos_b,
                               const Vector<3> thrust_dir_b, const double max_mag, const double mag_err, const double dir_err,
                               const Structure* structure, const Dynamics* dynamics)
    : Component(prescaler, clock_generator),
      id_(id),
      thruster_pos_b_(thruster_pos_b),
      thrust_dir_b_(thrust_dir_b),
      thrust_magnitude_max_(max_mag),
      thrust_dir_err_(dir_err),
      structure_(structure),
      dynamics_(dynamics) {
  Initialize(mag_err, dir_err);
}

SimpleThruster::SimpleThruster(const int prescaler, ClockGenerator* clock_generator, PowerPort* power_port, const int id,
                               const Vector<3> thruster_pos_b, const Vector<3> thrust_dir_b, const double max_mag, const double mag_err,
                               const double dir_err, const Structure* structure, const Dynamics* dynamics)
    : Component(prescaler, clock_generator, power_port),
      id_(id),
      thruster_pos_b_(thruster_pos_b),
      thrust_dir_b_(thrust_dir_b),
      thrust_magnitude_max_(max_mag),
      thrust_dir_err_(dir_err),
      structure_(structure),
      dynamics_(dynamics) {
  Initialize(mag_err, dir_err);
}

SimpleThruster::~SimpleThruster() {}

void SimpleThruster::Initialize(const double mag_err, const double dir_err) {
  mag_nr_.SetParameters(0.0, mag_err);
  dir_nr_.SetParameters(0.0, dir_err);
  thrust_dir_b_ = Normalize(thrust_dir_b_);
}

void SimpleThruster::MainRoutine(int count) {
  UNUSED(count);

  CalcThrust();
  CalcTorque(structure_->GetKinematicsParams().GetCGb());
}

void SimpleThruster::PowerOffRoutine() {
  thrust_b_ *= 0.0;
  torque_b_ *= 0.0;
}

void SimpleThruster::CalcThrust() {
  double mag = CalcThrustMagnitude();
  if (duty_ > 0.0 + DBL_EPSILON) mag += mag_nr_;
  thrust_b_ = mag * CalcThrustDir();
}

void SimpleThruster::CalcTorque(Vector<3> center) {
  Vector<3> vector_center2thruster = thruster_pos_b_ - center;
  Vector<3> torque = OuterProduct(vector_center2thruster, thrust_b_);

  torque_b_ = torque;
}

std::string SimpleThruster::GetLogHeader() const {
  std::string str_tmp = "";

  std::string head = "simple_thruster" + std::to_string(id_) + "_";
  str_tmp += WriteVector(head + "output_thrust", "b", "N", 3);
  str_tmp += WriteVector(head + "output_torque", "b", "Nm", 3);
  str_tmp += WriteScalar(head + "output_thrust_norm", "N");
  return str_tmp;
}

std::string SimpleThruster::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteVector(thrust_b_);
  str_tmp += WriteVector(torque_b_);
  str_tmp += WriteScalar(CalcNorm(thrust_b_));

  return str_tmp;
}

double SimpleThruster::CalcThrustMagnitude() { return duty_ * thrust_magnitude_max_; }

Vector<3> SimpleThruster::CalcThrustDir() {
  Vector<3> thrust_dir_b_true = thrust_dir_b_;
  if (thrust_dir_err_ > 0.0 + DBL_EPSILON) {
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

    Quaternion err_rot(axis_rot, dir_nr_);                           // Generate error quaternion
    thrust_dir_b_true = err_rot.FrameConversion(thrust_dir_b_true);  // Add error
  }

  return thrust_dir_b_true;
}
