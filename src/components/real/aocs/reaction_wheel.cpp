/*
 * @file reaction_wheel.cpp
 * @brief Class to emulate Reaction Wheel
 */
#include "reaction_wheel.hpp"

#include <fstream>
#include <iostream>
#include <library/initialize/initialize_file_access.hpp>
#include <library/math/constants.hpp>
#include <library/math/vector.hpp>
#include <random>

ReactionWheel::ReactionWheel(const int prescaler, const int fast_prescaler, ClockGenerator* clock_generator, const int component_id,
                             const double step_width_s, const double jitter_update_interval_s, const double rotor_inertia_kgm2,
                             const double max_torque_Nm, const double max_velocity_rpm, const libra::Quaternion quaternion_b2c,
                             const libra::Vector<3> position_b_m, const double dead_time_s, const bool is_calc_jitter_enabled,
                             const bool is_log_jitter_enabled, const std::vector<std::vector<double>> radial_force_harmonics_coefficients,
                             const std::vector<std::vector<double>> radial_torque_harmonics_coefficients,
                             const double structural_resonance_frequency_Hz, const double damping_factor, const double bandwidth,
                             const bool considers_structural_resonance, const bool drive_flag, const double init_velocity_rad_s)
    : Component(prescaler, clock_generator, fast_prescaler),
      component_id_(component_id),
      rotor_inertia_kgm2_(rotor_inertia_kgm2),
      max_torque_Nm_(max_torque_Nm),
      max_velocity_rpm_(max_velocity_rpm),
      quaternion_b2c_(quaternion_b2c),
      position_b_m_(position_b_m),
      step_width_s_(step_width_s),
      dead_time_s_(dead_time_s),
      drive_flag_(drive_flag),
      velocity_limit_rpm_(max_velocity_rpm_),
      ode_angular_velocity_(step_width_s_, velocity_limit_rpm_ * libra::rpm_to_rad_s, init_velocity_rad_s),
      rw_jitter_(radial_force_harmonics_coefficients, radial_torque_harmonics_coefficients, jitter_update_interval_s, quaternion_b2c,
                 structural_resonance_frequency_Hz, damping_factor, bandwidth, considers_structural_resonance),
      is_calculated_jitter_(is_calc_jitter_enabled),
      is_logged_jitter_(is_log_jitter_enabled) {
  Initialize();
}

ReactionWheel::ReactionWheel(const int prescaler, const int fast_prescaler, ClockGenerator* clock_generator, PowerPort* power_port,
                             const int component_id, const double step_width_s, const double jitter_update_interval_s,
                             const double rotor_inertia_kgm2, const double max_torque_Nm, const double max_velocity_rpm,
                             const libra::Quaternion quaternion_b2c, const libra::Vector<3> position_b_m, const double dead_time_s,
                             const bool is_calc_jitter_enabled, const bool is_log_jitter_enabled,
                             const std::vector<std::vector<double>> radial_force_harmonics_coefficients,
                             const std::vector<std::vector<double>> radial_torque_harmonics_coefficients,
                             const double structural_resonance_frequency_Hz, const double damping_factor, const double bandwidth,
                             const bool considers_structural_resonance, const bool drive_flag, const double init_velocity_rad_s)
    : Component(prescaler, clock_generator, power_port, fast_prescaler),
      component_id_(component_id),
      rotor_inertia_kgm2_(rotor_inertia_kgm2),
      max_torque_Nm_(max_torque_Nm),
      max_velocity_rpm_(max_velocity_rpm),
      quaternion_b2c_(quaternion_b2c),
      position_b_m_(position_b_m),
      step_width_s_(step_width_s),
      dead_time_s_(dead_time_s),
      drive_flag_(drive_flag),
      velocity_limit_rpm_(max_velocity_rpm_),
      ode_angular_velocity_(step_width_s, velocity_limit_rpm_ * libra::rpm_to_rad_s, init_velocity_rad_s),
      rw_jitter_(radial_force_harmonics_coefficients, radial_torque_harmonics_coefficients, jitter_update_interval_s, quaternion_b2c,
                 structural_resonance_frequency_Hz, damping_factor, bandwidth, considers_structural_resonance),
      is_calculated_jitter_(is_calc_jitter_enabled),
      is_logged_jitter_(is_log_jitter_enabled) {
  Initialize();
}

void ReactionWheel::Initialize() {
  rotation_axis_c_ = libra::Vector<3>(0.0);
  rotation_axis_c_[2] = 1.0;
  rotation_axis_b_ = quaternion_b2c_.InverseFrameConversion(rotation_axis_c_);

  // Acceleration
  target_acceleration_rad_s2_ = 0.0;
  int len_buffer = (int)floor(dead_time_s_ / step_width_s_) + 1;
  acceleration_delay_buffer_.assign(len_buffer, 0.0);
  generated_angular_acceleration_rad_s2_ = 0.0;

  angular_velocity_rad_s_ = ode_angular_velocity_.GetAngularVelocity_rad_s();
  angular_velocity_rpm_ = angular_velocity_rad_s_ * libra::rad_s_to_rpm;

  // Turn on RW jitter calculation
  if (is_calculated_jitter_) {
    SetNeedsFastUpdate(true);
  }
}

void ReactionWheel::MainRoutine(const int time_count) {
  UNUSED(time_count);

  CalcTorque();
}

void ReactionWheel::PowerOffRoutine() { drive_flag_ = 0; }

// Jitter calculation
void ReactionWheel::FastUpdate() { rw_jitter_.CalcJitter(angular_velocity_rad_s_); }

libra::Vector<3> ReactionWheel::CalcTorque() {
  double pre_angular_velocity_rad = angular_velocity_rad_s_;

  if (!drive_flag_)  // RW power off -> coasting mode
  {
    // Clear delay buffer
    std::fill(acceleration_delay_buffer_.begin(), acceleration_delay_buffer_.end(), 0.0);

    // Coasting torque
    double coasting_acceleration_rad_s2 = 1e-6;
    double rotation_direction = 1.0;
    if (angular_velocity_rad_s_ < 0.0) rotation_direction = -1.0;
    double angular_acceleration_rad_s2 = coasting_acceleration_rad_s2 * rotation_direction;
    ode_angular_velocity_.SetAngularAcceleration_rad_s2(angular_acceleration_rad_s2);
  } else  // RW power on
  {
    // Set target velocity from target torque
    double angular_acceleration_rad_s2 = acceleration_delay_buffer_.front();
    ode_angular_velocity_.SetAngularAcceleration_rad_s2(angular_acceleration_rad_s2);

    // Update delay buffer
    acceleration_delay_buffer_.push_back(target_acceleration_rad_s2_);
    acceleration_delay_buffer_.erase(acceleration_delay_buffer_.begin());
  }

  ++ode_angular_velocity_;  // propagate()

  // Substitution
  angular_velocity_rad_s_ = ode_angular_velocity_.GetAngularVelocity_rad_s();
  angular_velocity_rpm_ = angular_velocity_rad_s_ * libra::rad_s_to_rpm;
  generated_angular_acceleration_rad_s2_ = (angular_velocity_rad_s_ - pre_angular_velocity_rad) / step_width_s_;

  // Calc output torque by RW
  output_torque_b_Nm_ = -1.0 * rotor_inertia_kgm2_ * generated_angular_acceleration_rad_s2_ * rotation_axis_b_;
  angular_momentum_b_Nms_ = rotor_inertia_kgm2_ * angular_velocity_rad_s_ * rotation_axis_b_;
  return output_torque_b_Nm_;
}

const libra::Vector<3> ReactionWheel::GetOutputTorque_b_Nm() const {
  if (is_calculated_jitter_) {
    // Add jitter_force_b_N_-derived torque and jitter_torque_b_Nm_ to output_torque_b
    return output_torque_b_Nm_ - libra::OuterProduct(position_b_m_, rw_jitter_.GetJitterForce_b_N()) - rw_jitter_.GetJitterTorque_b_Nm();
  } else {
    return output_torque_b_Nm_;
  }
}

void ReactionWheel::SetTargetTorque_rw_Nm(const double torque_rw_Nm) {
  // Check Torque Limit
  double sign;
  torque_rw_Nm > 0 ? sign = 1.0 : sign = -1.0;
  if (abs(torque_rw_Nm) < max_torque_Nm_) {
    target_acceleration_rad_s2_ = torque_rw_Nm / rotor_inertia_kgm2_;
  } else {
    target_acceleration_rad_s2_ = sign * max_torque_Nm_ / rotor_inertia_kgm2_;
  }
}
void ReactionWheel::SetTargetTorque_b_Nm(const double torque_b_Nm) { SetTargetTorque_rw_Nm(-1.0 * torque_b_Nm); }

void ReactionWheel::SetVelocityLimit_rpm(const double velocity_limit_rpm) {
  if (velocity_limit_rpm > max_velocity_rpm_) {
    velocity_limit_rpm_ = max_velocity_rpm_;
  } else if (velocity_limit_rpm < -1.0 * max_velocity_rpm_) {
    velocity_limit_rpm_ = -1.0 * max_velocity_rpm_;
  } else {
    velocity_limit_rpm_ = velocity_limit_rpm;
  }
  ode_angular_velocity_.SetAngularVelocityLimit_rad_s(velocity_limit_rpm_ * libra::rpm_to_rad_s);
  return;
}

std::string ReactionWheel::GetLogHeader() const {
  std::string str_tmp = "";
  std::string component_name = "rw" + std::to_string(static_cast<long long>(component_id_)) + "_";

  str_tmp += WriteScalar(component_name + "angular_velocity", "rad/s");
  str_tmp += WriteScalar(component_name + "angular_velocity", "rpm");
  str_tmp += WriteScalar(component_name + "angular_velocity_upper_limit", "rpm");
  str_tmp += WriteScalar(component_name + "target_angular_acceleration", "rad/s2");
  str_tmp += WriteScalar(component_name + "angular_acceleration", "rad/s2");

  if (is_logged_jitter_) {
    str_tmp += WriteVector(component_name + "jitter_force", "c", "N", 3);
    str_tmp += WriteVector(component_name + "jitter_torque", "c", "Nm", 3);
  }

  return str_tmp;
}

std::string ReactionWheel::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteScalar(angular_velocity_rad_s_);
  str_tmp += WriteScalar(angular_velocity_rpm_);
  str_tmp += WriteScalar(velocity_limit_rpm_);
  str_tmp += WriteScalar(target_acceleration_rad_s2_);
  str_tmp += WriteScalar(generated_angular_acceleration_rad_s2_);

  if (is_logged_jitter_) {
    str_tmp += WriteVector(rw_jitter_.GetJitterForce_c_N());
    str_tmp += WriteVector(rw_jitter_.GetJitterTorque_c_Nm());
  }

  return str_tmp;
}

// In order to share processing among initialization functions, variables should also be shared.
// These variables have internal linkages and cannot be referenced from the outside.
namespace {
int prescaler;
int fast_prescaler;
double step_width_s;
double jitter_update_interval_s;
double rotor_inertia_kgm2;
double max_torque_Nm;
double max_velocity;
libra::Quaternion quaternion_b2c;
libra::Vector<3> position_b_m;
double dead_time_s;
bool is_calc_jitter_enabled;
bool is_log_jitter_enabled;
std::vector<std::vector<double>> radial_force_harmonics_coefficients;
std::vector<std::vector<double>> radial_torque_harmonics_coefficients;
double structural_resonance_frequency_Hz;
double damping_factor;
double bandwidth;
bool considers_structural_resonance;
bool drive_flag;
double init_velocity_rad_s;

void InitParams(int actuator_id, std::string file_name, double compo_update_step) {
  // Access Parameters
  IniAccess rw_ini_file(file_name);
  const std::string st_actuator_num = std::to_string(static_cast<long long>(actuator_id));
  const char* cs = st_actuator_num.data();
  std::string section_tmp = "REACTION_WHEEL_";
  section_tmp += cs;
  const char* RWsection = section_tmp.data();

  // Read ini file
  prescaler = rw_ini_file.ReadInt(RWsection, "prescaler");
  if (prescaler <= 1) prescaler = 1;
  fast_prescaler = rw_ini_file.ReadInt(RWsection, "fast_prescaler");
  if (fast_prescaler <= 1) fast_prescaler = 1;
  rotor_inertia_kgm2 = rw_ini_file.ReadDouble(RWsection, "moment_of_inertia_kgm2");
  max_torque_Nm = rw_ini_file.ReadDouble(RWsection, "max_output_torque_Nm");
  max_velocity = rw_ini_file.ReadDouble(RWsection, "max_angular_velocity_rpm");

  std::string direction_determination_mode;
  direction_determination_mode = rw_ini_file.ReadString(RWsection, "direction_determination_mode");
  if (direction_determination_mode == "QUATERNION") {
    rw_ini_file.ReadQuaternion(RWsection, "quaternion_b2c", quaternion_b2c);
  } else  // direction_determination_mode == "DIRECTION"
  {
    libra::Vector<3> direction_b;
    rw_ini_file.ReadVector(RWsection, "direction_b", direction_b);
    libra::Vector<3> direction_c(0.0);
    direction_c[2] = 1.0;
    libra::Quaternion q(direction_b, direction_c);
    quaternion_b2c = q.Conjugate();
  }

  rw_ini_file.ReadVector(RWsection, "position_b_m", position_b_m);
  dead_time_s = rw_ini_file.ReadDouble(RWsection, "dead_time_s");

  is_calc_jitter_enabled = rw_ini_file.ReadEnable(RWsection, "jitter_calculation");
  is_log_jitter_enabled = rw_ini_file.ReadEnable(RWsection, "jitter_logging");

  std::string radial_force_harmonics_coefficient_path = rw_ini_file.ReadString(RWsection, "radial_force_harmonics_coefficient_file");
  std::string radial_torque_harmonics_coefficient_path = rw_ini_file.ReadString(RWsection, "radial_torque_harmonics_coefficient_file");
  int harmonics_degree = rw_ini_file.ReadInt(RWsection, "harmonics_degree");
  IniAccess conf_radial_force_harmonics(radial_force_harmonics_coefficient_path);
  IniAccess conf_radial_torque_harmonics(radial_torque_harmonics_coefficient_path);
  conf_radial_force_harmonics.ReadCsvDouble(radial_force_harmonics_coefficients, harmonics_degree);
  conf_radial_torque_harmonics.ReadCsvDouble(radial_torque_harmonics_coefficients, harmonics_degree);

  structural_resonance_frequency_Hz = rw_ini_file.ReadDouble(RWsection, "structural_resonance_frequency_Hz");
  damping_factor = rw_ini_file.ReadDouble(RWsection, "damping_factor");
  bandwidth = rw_ini_file.ReadDouble(RWsection, "bandwidth");
  considers_structural_resonance = rw_ini_file.ReadEnable(RWsection, "considers_structural_resonance");

  drive_flag = rw_ini_file.ReadBoolean(RWsection, "initial_motor_drive_flag");
  init_velocity_rad_s = rw_ini_file.ReadDouble(RWsection, "initial_angular_velocity_rad_s");

  // Calc periods
  step_width_s = compo_update_step;
  jitter_update_interval_s = fast_prescaler * compo_update_step;
}
}  // namespace

ReactionWheel InitReactionWheel(ClockGenerator* clock_generator, int actuator_id, std::string file_name, double compo_update_step) {
  InitParams(actuator_id, file_name, compo_update_step);

  ReactionWheel rw(prescaler, fast_prescaler, clock_generator, actuator_id, step_width_s, jitter_update_interval_s, rotor_inertia_kgm2, max_torque_Nm,
                   max_velocity, quaternion_b2c, position_b_m, dead_time_s, is_calc_jitter_enabled, is_log_jitter_enabled,
                   radial_force_harmonics_coefficients, radial_torque_harmonics_coefficients, structural_resonance_frequency_Hz, damping_factor,
                   bandwidth, considers_structural_resonance, drive_flag, init_velocity_rad_s);

  return rw;
}

ReactionWheel InitReactionWheel(ClockGenerator* clock_generator, PowerPort* power_port, int actuator_id, std::string file_name,
                                double compo_update_step) {
  InitParams(actuator_id, file_name, compo_update_step);

  power_port->InitializeWithInitializeFile(file_name);

  ReactionWheel rw(prescaler, fast_prescaler, clock_generator, power_port, actuator_id, step_width_s, jitter_update_interval_s, rotor_inertia_kgm2,
                   max_torque_Nm, max_velocity, quaternion_b2c, position_b_m, dead_time_s, is_calc_jitter_enabled, is_log_jitter_enabled,
                   radial_force_harmonics_coefficients, radial_torque_harmonics_coefficients, structural_resonance_frequency_Hz, damping_factor,
                   bandwidth, considers_structural_resonance, drive_flag, init_velocity_rad_s);

  return rw;
}
