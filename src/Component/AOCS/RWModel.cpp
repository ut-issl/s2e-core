#include <random>
#include <fstream>
#include <iostream>
#include <Library/math/Constant.hpp>
#include <Library/math/Vector.hpp>
#include "RWModel.h"

using namespace libra;
using namespace std;

static double rpm2angularVelocity(double rpm)
{
  return rpm * libra::tau / 60.0;
}

static double angularVelocity2rpm(double angular_velocity)
{
  return angular_velocity * 60.0 / libra::tau;
}

RWModel::RWModel(
    int prescaler,
    int fast_prescaler,
    ClockGenerator *clock_gen,
    double step_width,
    double dt_main_routine,
    double jitter_update_interval,
    double inertia,
    double max_torque,
    double max_velocity_rpm,
    Quaternion q_b2c,
    Vector<3> pos_b,
    double dead_time,
    Vector<3> driving_lag_coef,
    Vector<3> coasting_lag_coef,
    bool is_calc_jitter_enabled,
    bool is_log_jitter_enabled,
    vector<vector<double>> radial_force_harmonics_coef,
    vector<vector<double>> radial_torque_harmonics_coef,
    double structural_resonance_freq,
    double damping_factor,
    double bandwidth,
    bool considers_structural_resonance,
    bool drive_flag,
    double init_velocity
    )
    : ComponentBase(prescaler, clock_gen, fast_prescaler),
      step_width_(step_width),
      dt_main_routine_(dt_main_routine),
      inertia_(inertia),
      max_torque_(max_torque),
      max_velocity_rpm_(max_velocity_rpm),
      q_b2c_(q_b2c),
      pos_b_(pos_b),
      dead_time_(dead_time),
      driving_lag_coef_(driving_lag_coef),
      coasting_lag_coef_(coasting_lag_coef),
      is_calculated_jitter_(is_calc_jitter_enabled),
      is_logged_jitter_(is_log_jitter_enabled),
      drive_flag_(drive_flag),
      ode_angular_velocity_(step_width_, init_velocity, 0.0, coasting_lag_coef_),
      rw_jitter_(radial_force_harmonics_coef, radial_torque_harmonics_coef, jitter_update_interval, q_b2c, structural_resonance_freq, damping_factor, bandwidth, considers_structural_resonance)
{
  Initialize();
}

RWModel::RWModel(
    int prescaler,
    int fast_prescaler,
    ClockGenerator *clock_gen,
    PowerPort* power_port,
    double step_width,
    double dt_main_routine,
    double jitter_update_interval,
    double inertia,
    double max_torque,
    double max_velocity_rpm,
    Quaternion q_b2c,
    Vector<3> pos_b,
    double dead_time,
    Vector<3> driving_lag_coef,
    Vector<3> coasting_lag_coef,
    bool is_calc_jitter_enabled,
    bool is_log_jitter_enabled,
    vector<vector<double>> radial_force_harmonics_coef,
    vector<vector<double>> radial_torque_harmonics_coef,
    double structural_resonance_freq,
    double damping_factor,
    double bandwidth,
    bool considers_structural_resonance,
    bool drive_flag,
    double init_velocity)
    : ComponentBase(prescaler, clock_gen, power_port, fast_prescaler),
      step_width_(step_width),
      dt_main_routine_(dt_main_routine),
      inertia_(inertia),
      max_torque_(max_torque),
      max_velocity_rpm_(max_velocity_rpm),
      q_b2c_(q_b2c),
      pos_b_(pos_b),
      dead_time_(dead_time),
      driving_lag_coef_(driving_lag_coef),
      coasting_lag_coef_(coasting_lag_coef),
      is_calculated_jitter_(is_calc_jitter_enabled),
      is_logged_jitter_(is_log_jitter_enabled),
      drive_flag_(drive_flag),
      ode_angular_velocity_(step_width_, init_velocity, 0.0, coasting_lag_coef_),
      rw_jitter_(radial_force_harmonics_coef, radial_torque_harmonics_coef, jitter_update_interval, q_b2c, structural_resonance_freq, damping_factor, bandwidth, considers_structural_resonance)
{
  Initialize();
}

void RWModel::Initialize()
{
  direction_c_ = Vector<3>(0.0); direction_c_[2] = 1.0;
  direction_b_ = q_b2c_.frame_conv_inv(direction_c_);

  velocity_limit_rpm_ = max_velocity_rpm_;
  output_torque_b_ = Vector<3>(0.0);
  angular_momentum_b_ = Vector<3>(0.0);
  target_accl_ = 0.0;
  int len_buffer = floor(dead_time_ / dt_main_routine_) + 1;
  delay_buffer_accl_.assign(len_buffer, 0.0);

  angular_acceleration_ = 0.0;
  angular_velocity_rpm_ = 0.0;
  angular_velocity_rad_ = 0.0;

  //Turn on RW jitter calculation
  if (is_calculated_jitter_) { SetNeedsFastUpdate(true);}
}

void RWModel::MainRoutine(int count)
{
  CalcTorque();
}

//Jitter calculation
void RWModel::FastUpdate()
{
  rw_jitter_.CalcJitter(angular_velocity_rad_);
}

Vector<3> RWModel::CalcTorque()
{
  double pre_angular_velocity_rad = angular_velocity_rad_;
  if (!drive_flag_) //RW power off -> coasting mode
  {
    //Set lag coefficient
    ode_angular_velocity_.setLagCoef(coasting_lag_coef_);
    //Set target velocity
    ode_angular_velocity_.setTargetAngularVelocity(0.0);
    //Clear delay buffer
    std::fill(delay_buffer_accl_.begin(), delay_buffer_accl_.end(), 0.0);
  }
  else //RW power on
  {
    //Set lag coefficient
    ode_angular_velocity_.setLagCoef(driving_lag_coef_);
    //Set target velocity from target torque
    double angular_accl = delay_buffer_accl_.front();
    double target_angular_velocity_rad = pre_angular_velocity_rad + angular_accl;
    //Check velocity limit
    double velocity_limit_rad = rpm2angularVelocity(velocity_limit_rpm_);
    if (target_angular_velocity_rad > velocity_limit_rad)
      target_angular_velocity_rad = velocity_limit_rad;
    else if (target_angular_velocity_rad < -1.0 * velocity_limit_rad)
      target_angular_velocity_rad = -1.0 * velocity_limit_rad;
    //Set target velocity
    ode_angular_velocity_.setTargetAngularVelocity(target_angular_velocity_rad);
    //Update delay buffer
    delay_buffer_accl_.push_back(target_accl_);
    delay_buffer_accl_.erase(delay_buffer_accl_.begin());
  }
  //Calc RW ODE
  int itr_num = ceil(dt_main_routine_ / step_width_);
  for (int i = 0; i < itr_num; i++)
  {
    ++ode_angular_velocity_; // propagate()
  }
  //Substitution
  angular_velocity_rad_ = ode_angular_velocity_.getAngularVelocity();
  angular_velocity_rpm_ = angularVelocity2rpm(angular_velocity_rad_);
  angular_acceleration_ = (angular_velocity_rad_ - pre_angular_velocity_rad) / dt_main_routine_;
  //Component frame -> Body frame
  output_torque_b_ = -1.0 * inertia_ * angular_acceleration_ * direction_b_;
  angular_momentum_b_ = inertia_ * angular_velocity_rad_ * direction_b_;
  return output_torque_b_;
}

const libra::Vector<3> RWModel::GetOutputTorqueB() const
{
  if (is_calculated_jitter_)
  {
    //Add jitter_force_b_-derived torque and jitter_torque_b_ to output_torqur_b
    return output_torque_b_ - libra::outer_product(pos_b_, rw_jitter_.GetJitterForceB()) - rw_jitter_.GetJitterTorqueB();
  }
  else
  {
    return output_torque_b_;
  }
}

void RWModel::SetTargetTorqueRw(double torque_rw)
{
  // Check Torque Limit
  double sign;
  torque_rw > 0 ? sign = 1.0 : sign = -1.0;
  if (abs(torque_rw) < max_torque_)
  {
    target_accl_ = torque_rw / inertia_;
  }
  else
  {
    target_accl_ = sign * max_torque_ / inertia_;
  }
}
void RWModel::SetTargetTorqueBody(double torque_body)
{
  SetTargetTorqueRw(-1.0*torque_body);
}

void RWModel::SetVelocityLimitRpm(double velocity_limit_rpm)
{
  if (velocity_limit_rpm > max_velocity_rpm_)
  {
    velocity_limit_rpm_ = max_velocity_rpm_;
  }
  else if (velocity_limit_rpm < -1.0 * max_velocity_rpm_)
  {
    velocity_limit_rpm_ = -1.0 * max_velocity_rpm_;
  }
  else
  {
    velocity_limit_rpm_ = velocity_limit_rpm;
  }
  return;
}

std::string RWModel::GetLogHeader() const
{
  std::string str_tmp = "";

  str_tmp += WriteScalar("rw_angular_velocity", "rad/s");
  str_tmp += WriteScalar("rw_angular_velocity_rpm", "rpm");
  str_tmp += WriteScalar("rw_angular_velocity_upperlimit", "rpm");
  str_tmp += WriteScalar("rw_angular_acceleration", "rad/s^2");

  if (is_logged_jitter_)
  {
    str_tmp += WriteVector("rw_jitter_force", "c", "N", 3);
    str_tmp += WriteVector("rw_jitter_torque", "c", "Nm", 3);
  }

  return str_tmp;
}

std::string RWModel::GetLogValue() const
{
  std::string str_tmp = "";

  str_tmp += WriteScalar(angular_velocity_rad_);
  str_tmp += WriteScalar(angular_velocity_rpm_);
  str_tmp += WriteScalar(velocity_limit_rpm_);
  str_tmp += WriteScalar(angular_acceleration_);

  if (is_logged_jitter_)
  {
    str_tmp += WriteVector(rw_jitter_.GetJitterForceC());
    str_tmp += WriteVector(rw_jitter_.GetJitterTorqueC());
  }

  return str_tmp;
}
