/*
 * @file reaction_wheel.cpp
 * @brief Class to emulate Reaction Wheel
 */
#include "reaction_wheel.hpp"

#include <fstream>
#include <iostream>
#include <library/math/constants.hpp>
#include <library/math/vector.hpp>
#include <random>

using namespace libra;
using namespace std;

static double rpm2angularVelocity(double rpm) { return rpm * libra::tau / 60.0; }

static double angularVelocity2rpm(double angular_velocity) { return angular_velocity * 60.0 / libra::tau; }

RWModel::RWModel(int prescaler, int fast_prescaler, ClockGenerator *clock_generator, const int component_id, double step_width_s,
                 double main_routine_time_step_s, double jitter_update_interval_s, double rotor_inertia_kgm2, double max_torque_Nm,
                 double max_velocity_rpm, libra::Quaternion quaternion_b2c, libra::Vector<3> position_b_m, double dead_time_s,
                 libra::Vector<3> driving_lag_coefficients, libra::Vector<3> coasting_lag_coefficients, bool is_calc_jitter_enabled,
                 bool is_log_jitter_enabled, vector<vector<double>> radial_force_harmonics_coefficients,
                 vector<vector<double>> radial_torque_harmonics_coefficients, double structural_resonance_frequency_Hz, double damping_factor,
                 double bandwidth, bool considers_structural_resonance, bool drive_flag, double init_velocity_rad_s)
    : Component(prescaler, clock_generator, fast_prescaler),
      component_id_(component_id),
      rotor_inertia_kgm2_(rotor_inertia_kgm2),
      max_torque_Nm_(max_torque_Nm),
      max_velocity_rpm_(max_velocity_rpm),
      quaternion_b2c_(quaternion_b2c),
      position_b_m_(position_b_m),
      step_width_s_(step_width_s),
      dead_time_s_(dead_time_s),
      driving_lag_coefficients_(driving_lag_coefficients),
      coasting_lag_coefficients_(coasting_lag_coefficients),
      drive_flag_(drive_flag),
      main_routine_time_step_s_(main_routine_time_step_s),
      ode_angular_velocity_(step_width_s_, init_velocity_rad_s, 0.0, coasting_lag_coefficients_),
      rw_jitter_(radial_force_harmonics_coefficients, radial_torque_harmonics_coefficients, jitter_update_interval_s, quaternion_b2c,
                 structural_resonance_frequency_Hz, damping_factor, bandwidth, considers_structural_resonance),
      is_calculated_jitter_(is_calc_jitter_enabled),
      is_logged_jitter_(is_log_jitter_enabled) {
  Initialize();
}

RWModel::RWModel(int prescaler, int fast_prescaler, ClockGenerator *clock_generator, PowerPort *power_port, const int component_id,
                 double step_width_s, double main_routine_time_step_s, double jitter_update_interval_s, double rotor_inertia_kgm2,
                 double max_torque_Nm, double max_velocity_rpm, libra::Quaternion quaternion_b2c, libra::Vector<3> position_b_m, double dead_time_s,
                 libra::Vector<3> driving_lag_coefficients, libra::Vector<3> coasting_lag_coefficients, bool is_calc_jitter_enabled,
                 bool is_log_jitter_enabled, vector<vector<double>> radial_force_harmonics_coefficients,
                 vector<vector<double>> radial_torque_harmonics_coefficients, double structural_resonance_frequency_Hz, double damping_factor,
                 double bandwidth, bool considers_structural_resonance, bool drive_flag, double init_velocity_rad_s)
    : Component(prescaler, clock_generator, power_port, fast_prescaler),
      component_id_(component_id),
      rotor_inertia_kgm2_(rotor_inertia_kgm2),
      max_torque_Nm_(max_torque_Nm),
      max_velocity_rpm_(max_velocity_rpm),
      quaternion_b2c_(quaternion_b2c),
      position_b_m_(position_b_m),
      step_width_s_(step_width_s),
      dead_time_s_(dead_time_s),
      driving_lag_coefficients_(driving_lag_coefficients),
      coasting_lag_coefficients_(coasting_lag_coefficients),
      drive_flag_(drive_flag),
      main_routine_time_step_s_(main_routine_time_step_s),
      ode_angular_velocity_(step_width_s_, init_velocity_rad_s, 0.0, coasting_lag_coefficients_),
      rw_jitter_(radial_force_harmonics_coefficients, radial_torque_harmonics_coefficients, jitter_update_interval_s, quaternion_b2c,
                 structural_resonance_frequency_Hz, damping_factor, bandwidth, considers_structural_resonance),
      is_calculated_jitter_(is_calc_jitter_enabled),
      is_logged_jitter_(is_log_jitter_enabled) {
  Initialize();
}

void RWModel::Initialize() {
  rotation_axis_c_ = libra::Vector<3>(0.0);
  rotation_axis_c_[2] = 1.0;
  rotation_axis_b_ = quaternion_b2c_.InverseFrameConversion(rotation_axis_c_);

  velocity_limit_rpm_ = max_velocity_rpm_;
  output_torque_b_Nm_ = libra::Vector<3>(0.0);
  angular_momentum_b_Nms_ = libra::Vector<3>(0.0);
  target_acceleration_rad_s2_ = 0.0;
  int len_buffer = (int)floor(dead_time_s_ / main_routine_time_step_s_) + 1;
  acceleration_delay_buffer_.assign(len_buffer, 0.0);

  angular_acceleration_rad_s2_ = 0.0;
  angular_velocity_rpm_ = 0.0;
  angular_velocity_rad_s_ = 0.0;

  // Turn on RW jitter calculation
  if (is_calculated_jitter_) {
    SetNeedsFastUpdate(true);
  }
}

void RWModel::MainRoutine(const int time_count) {
  UNUSED(time_count);

  CalcTorque();
}

void RWModel::PowerOffRoutine() { drive_flag_ = 0; }

// Jitter calculation
void RWModel::FastUpdate() { rw_jitter_.CalcJitter(angular_velocity_rad_s_); }

libra::Vector<3> RWModel::CalcTorque() {
  double pre_angular_velocity_rad = angular_velocity_rad_s_;
  if (!drive_flag_)  // RW power off -> coasting mode
  {
    // Set lag coefficient
    ode_angular_velocity_.SetLagCoefficients(coasting_lag_coefficients_);
    // Set target velocity
    ode_angular_velocity_.SetTargetAngularVelocity_rad_s(0.0);
    // Clear delay buffer
    std::fill(acceleration_delay_buffer_.begin(), acceleration_delay_buffer_.end(), 0.0);
  } else  // RW power on
  {
    // Set lag coefficient
    ode_angular_velocity_.SetLagCoefficients(driving_lag_coefficients_);
    // Set target velocity from target torque
    double angular_accl = acceleration_delay_buffer_.front();
    double target_angular_velocity_rad = pre_angular_velocity_rad + angular_accl;
    // Check velocity limit
    double velocity_limit_rad = rpm2angularVelocity(velocity_limit_rpm_);
    if (target_angular_velocity_rad > velocity_limit_rad)
      target_angular_velocity_rad = velocity_limit_rad;
    else if (target_angular_velocity_rad < -1.0 * velocity_limit_rad)
      target_angular_velocity_rad = -1.0 * velocity_limit_rad;
    // Set target velocity
    ode_angular_velocity_.SetTargetAngularVelocity_rad_s(target_angular_velocity_rad);
    // Update delay buffer
    acceleration_delay_buffer_.push_back(target_acceleration_rad_s2_);
    acceleration_delay_buffer_.erase(acceleration_delay_buffer_.begin());
  }
  // Calc RW OrdinaryDifferentialEquation
  int itr_num = (int)ceil(main_routine_time_step_s_ / step_width_s_);
  for (int i = 0; i < itr_num; i++) {
    ++ode_angular_velocity_;  // propagate()
  }
  // Substitution
  angular_velocity_rad_s_ = ode_angular_velocity_.GetAngularVelocity_rad_s();
  angular_velocity_rpm_ = angularVelocity2rpm(angular_velocity_rad_s_);
  angular_acceleration_rad_s2_ = (angular_velocity_rad_s_ - pre_angular_velocity_rad) / main_routine_time_step_s_;
  // Component frame -> Body frame
  output_torque_b_Nm_ = -1.0 * rotor_inertia_kgm2_ * angular_acceleration_rad_s2_ * rotation_axis_b_;
  angular_momentum_b_Nms_ = rotor_inertia_kgm2_ * angular_velocity_rad_s_ * rotation_axis_b_;
  return output_torque_b_Nm_;
}

const libra::Vector<3> RWModel::GetOutputTorqueB() const {
  if (is_calculated_jitter_) {
    // Add jitter_force_b_N_-derived torque and jitter_torque_b_Nm_ to output_torque_b
    return output_torque_b_Nm_ - libra::OuterProduct(position_b_m_, rw_jitter_.GetJitterForce_b_N()) - rw_jitter_.GetJitterTorque_b_Nm();
  } else {
    return output_torque_b_Nm_;
  }
}

void RWModel::SetTargetTorqueRw(double torque_rw) {
  // Check Torque Limit
  double sign;
  torque_rw > 0 ? sign = 1.0 : sign = -1.0;
  if (abs(torque_rw) < max_torque_Nm_) {
    target_acceleration_rad_s2_ = torque_rw / rotor_inertia_kgm2_;
  } else {
    target_acceleration_rad_s2_ = sign * max_torque_Nm_ / rotor_inertia_kgm2_;
  }
}
void RWModel::SetTargetTorqueBody(double torque_body) { SetTargetTorqueRw(-1.0 * torque_body); }

void RWModel::SetVelocityLimitRpm(double velocity_limit_rpm) {
  if (velocity_limit_rpm > max_velocity_rpm_) {
    velocity_limit_rpm_ = max_velocity_rpm_;
  } else if (velocity_limit_rpm < -1.0 * max_velocity_rpm_) {
    velocity_limit_rpm_ = -1.0 * max_velocity_rpm_;
  } else {
    velocity_limit_rpm_ = velocity_limit_rpm;
  }
  return;
}

std::string RWModel::GetLogHeader() const {
  std::string str_tmp = "";
  std::string component_name = "rw" + std::to_string(static_cast<long long>(component_id_)) + "_";

  str_tmp += WriteScalar(component_name + "angular_velocity", "rad/s");
  str_tmp += WriteScalar(component_name + "angular_velocity", "rpm");
  str_tmp += WriteScalar(component_name + "angular_velocity_upper_limit", "rpm");
  str_tmp += WriteScalar(component_name + "angular_acceleration", "rad/s2");

  if (is_logged_jitter_) {
    str_tmp += WriteVector(component_name + "jitter_force", "c", "N", 3);
    str_tmp += WriteVector(component_name + "jitter_torque", "c", "Nm", 3);
  }

  return str_tmp;
}

std::string RWModel::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteScalar(angular_velocity_rad_s_);
  str_tmp += WriteScalar(angular_velocity_rpm_);
  str_tmp += WriteScalar(velocity_limit_rpm_);
  str_tmp += WriteScalar(angular_acceleration_rad_s2_);

  if (is_logged_jitter_) {
    str_tmp += WriteVector(rw_jitter_.GetJitterForce_c_N());
    str_tmp += WriteVector(rw_jitter_.GetJitterTorque_c_Nm());
  }

  return str_tmp;
}
