#include "SimpleThruster.h"

#include <Library/math/GlobalRand.h>

#include <Library/math/Constant.hpp>
#include <cfloat>

// Constructor
SimpleThruster::SimpleThruster(const int prescaler, ClockGenerator* clock_gen, const int id, const Vector<3> thruster_pos_b,
                               const Vector<3> thrust_dir_b, const double max_mag, const double mag_err, const double dir_err,
                               const Structure* structure, const Dynamics* dynamics)
    : ComponentBase(prescaler, clock_gen),
      id_(id),
      thruster_pos_b_(thruster_pos_b),
      thrust_dir_b_(thrust_dir_b),
      thrust_magnitude_max_(max_mag),
      thrust_dir_err_(dir_err),
      structure_(structure),
      dynamics_(dynamics) {
  Initialize(mag_err, dir_err);
}

SimpleThruster::SimpleThruster(const int prescaler, ClockGenerator* clock_gen, PowerPort* power_port, const int id, const Vector<3> thruster_pos_b,
                               const Vector<3> thrust_dir_b, const double max_mag, const double mag_err, const double dir_err,
                               const Structure* structure, const Dynamics* dynamics)
    : ComponentBase(prescaler, clock_gen, power_port),
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
  mag_nr_.set_param(0.0, mag_err);
  dir_nr_.set_param(0.0, dir_err);
  thrust_dir_b_ = normalize(thrust_dir_b_);
}

void SimpleThruster::MainRoutine(int count) {
  UNUSED(count);

  CalcThrust();
  CalcTorque(structure_->GetKinematicsParams().GetCGb());
}

void SimpleThruster::CalcThrust() {
  double mag = CalcThrustMagnitude();
  if (duty_ > 0.0 + DBL_EPSILON) mag += mag_nr_;
  thrust_b_ = mag * CalcThrustDir();
}

void SimpleThruster::CalcTorque(Vector<3> center) {
  Vector<3> vector_center2thruster = thruster_pos_b_ - center;
  Vector<3> torque = outer_product(vector_center2thruster, thrust_b_);

  torque_b_ = torque;
}

std::string SimpleThruster::GetLogHeader() const {
  std::string str_tmp = "";

  std::string head = "TH" + std::to_string(id_);
  str_tmp += WriteVector(head + "thrust", "b", "N", 3);
  str_tmp += WriteVector(head + "torque", "b", "Nm", 3);
  str_tmp += WriteScalar(head + "thrust", "N");
  return str_tmp;
}

std::string SimpleThruster::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteVector(thrust_b_);
  str_tmp += WriteVector(torque_b_);
  str_tmp += WriteScalar(norm(thrust_b_));

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
    Vector<3> axis_rot = make_axis_rot.frame_conv(ex);

    Quaternion err_rot(axis_rot, dir_nr_);                      // Generate error quaternion
    thrust_dir_b_true = err_rot.frame_conv(thrust_dir_b_true);  // Add error
  }

  return thrust_dir_b_true;
}
