/*
 * @file simple_thruster.cpp
 * @brief Component emulation of simple thruster
 */
#include "simple_thruster.hpp"

#include <cfloat>
#include <math_physics/math/constants.hpp>
#include <math_physics/randomization/global_randomization.hpp>
#include <setting_file_reader/initialize_file_access.hpp>

namespace s2e::components {

// Constructor
SimpleThruster::SimpleThruster(const int prescaler, environment::ClockGenerator* clock_generator, const int component_id,
                               const math::Vector<3> thruster_position_b_m, const math::Vector<3> thrust_direction_b, const double max_magnitude_N,
                               const double magnitude_standard_deviation_N, const double direction_standard_deviation_rad, const double dead_time_s,
                               const double time_constant_s, const double step_width_s, const spacecraft::Structure* structure,
                               const dynamics::Dynamics* dynamics)
    : Component(prescaler, clock_generator),
      component_id_(component_id),
      thruster_position_b_m_(thruster_position_b_m),
      thrust_direction_b_(thrust_direction_b),
      thrust_magnitude_max_N_(max_magnitude_N),
      direction_noise_standard_deviation_rad_(direction_standard_deviation_rad),
      dead_time_s_(dead_time_s),
      step_width_s_(step_width_s),
      delayed_duty_(step_width_s, time_constant_s),
      structure_(structure),
      dynamics_(dynamics) {
  Initialize(magnitude_standard_deviation_N, direction_standard_deviation_rad);
  InitializeDelay();
}

SimpleThruster::SimpleThruster(const int prescaler, environment::ClockGenerator* clock_generator, PowerPort* power_port, const int component_id,
                               const math::Vector<3> thruster_position_b_m, const math::Vector<3> thrust_direction_b, const double max_magnitude_N,
                               const double magnitude_standard_deviation_N, const double direction_standard_deviation_rad, const double dead_time_s,
                               const double time_constant_s, const double step_width_s, const spacecraft::Structure* structure,
                               const dynamics::Dynamics* dynamics)
    : Component(prescaler, clock_generator, power_port),
      component_id_(component_id),
      thruster_position_b_m_(thruster_position_b_m),
      thrust_direction_b_(thrust_direction_b),
      thrust_magnitude_max_N_(max_magnitude_N),
      direction_noise_standard_deviation_rad_(direction_standard_deviation_rad),
      dead_time_s_(dead_time_s),
      step_width_s_(step_width_s),
      delayed_duty_(step_width_s, time_constant_s),
      structure_(structure),
      dynamics_(dynamics) {
  Initialize(magnitude_standard_deviation_N, direction_standard_deviation_rad);
  InitializeDelay();
}

SimpleThruster::~SimpleThruster() {}

void SimpleThruster::Initialize(const double magnitude_standard_deviation_N, const double direction_standard_deviation_rad) {
  magnitude_random_noise_.SetParameters(0.0, magnitude_standard_deviation_N);
  direction_random_noise_.SetParameters(0.0, direction_standard_deviation_rad);
  thrust_direction_b_ = thrust_direction_b_.CalcNormalizedVector();
}

void SimpleThruster::InitializeDelay() {
  size_t len_buffer = (size_t)floor(dead_time_s_ / step_width_s_) + 1;
  duty_delay_buffer_.assign(len_buffer, 0.0);
}

void SimpleThruster::MainRoutine(const int time_count) {
  UNUSED(time_count);

  CalcThrust();
  CalcTorque(structure_->GetKinematicsParameters().GetCenterOfGravity_b_m());
}

void SimpleThruster::PowerOffRoutine() {
  output_thrust_b_N_ *= 0.0;
  output_torque_b_Nm_ *= 0.0;
}

void SimpleThruster::CalcThrust() {
  double mag = CalcThrustMagnitude();
  if (duty_ > 0.0 + DBL_EPSILON) mag += magnitude_random_noise_;
  output_thrust_b_N_ = mag * CalcThrustDirection();
}

void SimpleThruster::CalcTorque(const math::Vector<3> center_of_mass_b_m) {
  math::Vector<3> vector_center2thruster = thruster_position_b_m_ - center_of_mass_b_m;
  math::Vector<3> torque = OuterProduct(vector_center2thruster, output_thrust_b_N_);

  output_torque_b_Nm_ = torque;
}

std::string SimpleThruster::GetLogHeader() const {
  std::string str_tmp = "";

  std::string head = "simple_thruster" + std::to_string(component_id_) + "_";
  str_tmp += logger::WriteVector(head + "output_thrust", "b", "N", 3);
  str_tmp += logger::WriteVector(head + "output_torque", "b", "Nm", 3);
  str_tmp += logger::WriteScalar(head + "output_thrust_norm", "N");
  return str_tmp;
}

std::string SimpleThruster::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += logger::WriteVector(output_thrust_b_N_);
  str_tmp += logger::WriteVector(output_torque_b_Nm_);
  str_tmp += logger::WriteScalar(output_thrust_b_N_.CalcNorm());

  return str_tmp;
}

double SimpleThruster::CalcThrustMagnitude() {
  // Get delayed duty from dead time buffer
  double delayed_duty = duty_delay_buffer_.front();
  duty_delay_buffer_.push_back(duty_);
  duty_delay_buffer_.erase(duty_delay_buffer_.begin());

  // Apply first-order lag
  delayed_duty = delayed_duty_.Update(delayed_duty);

  return delayed_duty * thrust_magnitude_max_N_;
}

math::Vector<3> SimpleThruster::CalcThrustDirection() {
  math::Vector<3> thrust_dir_b_true = thrust_direction_b_;
  if (direction_noise_standard_deviation_rad_ > 0.0 + DBL_EPSILON) {
    math::Vector<3> ex;  // Fixme: to use outer product to generate orthogonal vector
    ex[0] = 1.0;
    ex[1] = 0.0;
    ex[2] = 0.0;
    int flag = rand() % 2;
    double make_axis_rot_rad;
    if (flag == 0) {
      make_axis_rot_rad = math::pi * (double)rand() / RAND_MAX;
    } else {
      make_axis_rot_rad = -math::pi * (double)rand() / RAND_MAX;
    }

    math::Quaternion make_axis_rot(thrust_dir_b_true, make_axis_rot_rad);
    math::Vector<3> axis_rot = make_axis_rot.FrameConversion(ex);

    math::Quaternion err_rot(axis_rot, direction_random_noise_);     // Generate error quaternion
    thrust_dir_b_true = err_rot.FrameConversion(thrust_dir_b_true);  // Add error
  }

  return thrust_dir_b_true;
}

SimpleThruster InitSimpleThruster(environment::ClockGenerator* clock_generator, int thruster_id, const std::string file_name,
                                  const spacecraft::Structure* structure, const dynamics::Dynamics* dynamics) {
  setting_file_reader::IniAccess thruster_conf(file_name);
  std::string section_str = "THRUSTER_" + std::to_string(thruster_id);
  auto* Section = section_str.c_str();

  int prescaler = thruster_conf.ReadInt(Section, "prescaler");
  if (prescaler <= 1) prescaler = 1;

  math::Vector<3> thruster_pos;
  thruster_conf.ReadVector(Section, "thruster_position_b_m", thruster_pos);

  math::Vector<3> thruster_dir;
  thruster_conf.ReadVector(Section, "thruster_direction_b", thruster_dir);

  double max_magnitude_N = thruster_conf.ReadDouble(Section, "thrust_magnitude_N");

  double magnitude_standard_deviation_N;
  magnitude_standard_deviation_N = thruster_conf.ReadDouble(Section, "thrust_error_standard_deviation_N");

  double deg_err;
  deg_err = thruster_conf.ReadDouble(Section, "direction_error_standard_deviation_deg") * math::pi / 180.0;

  // Read delay parameters
  double dead_time_s = thruster_conf.ReadDouble(Section, "dead_time_s");
  double time_constant_s = thruster_conf.ReadDouble(Section, "time_constant_s");
  double step_width_s = thruster_conf.ReadDouble(Section, "step_width_s");

  SimpleThruster thruster(prescaler, clock_generator, thruster_id, thruster_pos, thruster_dir, max_magnitude_N, magnitude_standard_deviation_N,
                          deg_err, dead_time_s, time_constant_s, step_width_s, structure, dynamics);
  return thruster;
}

SimpleThruster InitSimpleThruster(environment::ClockGenerator* clock_generator, PowerPort* power_port, int thruster_id, const std::string file_name,
                                  const spacecraft::Structure* structure, const dynamics::Dynamics* dynamics) {
  setting_file_reader::IniAccess thruster_conf(file_name);
  std::string section_str = "THRUSTER_" + std::to_string(thruster_id);
  auto* Section = section_str.c_str();

  int prescaler = thruster_conf.ReadInt(Section, "prescaler");
  if (prescaler <= 1) prescaler = 1;

  math::Vector<3> thruster_pos;
  thruster_conf.ReadVector(Section, "thruster_position_b_m", thruster_pos);

  math::Vector<3> thruster_dir;
  thruster_conf.ReadVector(Section, "thruster_direction_b", thruster_dir);

  double max_magnitude_N = thruster_conf.ReadDouble(Section, "thrust_magnitude_N");

  double magnitude_standard_deviation_N;
  magnitude_standard_deviation_N = thruster_conf.ReadDouble(Section, "thrust_error_standard_deviation_N");

  double deg_err;
  deg_err = thruster_conf.ReadDouble(Section, "direction_error_standard_deviation_deg") * math::pi / 180.0;

  // Read delay parameters
  double dead_time_s = thruster_conf.ReadDouble(Section, "dead_time_s");
  double time_constant_s = thruster_conf.ReadDouble(Section, "time_constant_s");
  double step_width_s = thruster_conf.ReadDouble(Section, "step_width_s");

  power_port->InitializeWithInitializeFile(file_name);

  SimpleThruster thruster(prescaler, clock_generator, power_port, thruster_id, thruster_pos, thruster_dir, max_magnitude_N,
                          magnitude_standard_deviation_N, deg_err, dead_time_s, time_constant_s, step_width_s, structure, dynamics);
  return thruster;
}

}  // namespace s2e::components
