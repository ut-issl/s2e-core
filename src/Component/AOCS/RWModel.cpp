#define _USE_MATH_DEFINES
#include <cmath>
#include <fstream>
#include <iostream>
#include "../../Library/math/Vector.hpp"
#include "RWModel.h"

using namespace libra;
using namespace std;

static double rpm2angularVelocity(double rpm)
{
  return rpm * 2.0 * M_PI / 60.0;
}

static double angularVelocity2rpm(double angular_velocity)
{
  return angular_velocity * 60.0 / (2.0 * M_PI);
}

RWModel::RWModel(
    int prescaler,
    ClockGenerator *clock_gen,
    double step_width,
    double dt_main_routine,
    double inertia,
    double max_torque,
    double max_velocity_rpm,
    Vector<3> direction_b,
    double dead_time,
    Vector<3> driving_lag_coef,
    Vector<3> coasting_lag_coef,
    bool drive_flag,
    double init_velocity)
    : ComponentBase(prescaler, clock_gen),
      step_width_(step_width),
      dt_main_routine_(dt_main_routine),
      inertia_(inertia),
      max_torque_(max_torque),
      max_velocity_rpm_(max_velocity_rpm),
      direction_b_(direction_b),
      dead_time_(dead_time),
      driving_lag_coef_(driving_lag_coef),
      coasting_lag_coef_(coasting_lag_coef),
      drive_flag_(drive_flag),
      ode_angular_velocity_(step_width_, init_velocity, 0.0, coasting_lag_coef_)
{
  velocity_limit_rpm_ = max_velocity_rpm_;
  output_torque_b_ = Vector<3>(0.0);
  angular_momentum_b_ = Vector<3>(0.0);
  target_accl_ = 0.0;
  int len_buffer = floor(dead_time_ / dt_main_routine_) + 1;
  delay_buffer_accl_.assign(len_buffer, 0.0);

  angular_acceleration_ = 0.0;
  angular_velocity_rpm_ = 0.0;
  angular_velocity_rad_ = 0.0;
}

void RWModel::MainRoutine(int count)
{
  CalcTorque();
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
//制御時の最大角速度設定
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

string RWModel::GetLogHeader() const
{
  string str_tmp = "";

  str_tmp += WriteScalar("rw_angular_velocity", "rad/s");
  str_tmp += WriteScalar("rw_angular_velocity_rpm", "rpm");
  str_tmp += WriteScalar("rw_angular_velocity_upperlimit", "rpm");
  str_tmp += WriteScalar("rw_angular_acceleration", "rad/s^2");

  return str_tmp;
}

string RWModel::GetLogValue() const
{
  string str_tmp = "";

  str_tmp += WriteScalar(angular_velocity_rad_);
  str_tmp += WriteScalar(angular_velocity_rpm_);
  str_tmp += WriteScalar(velocity_limit_rpm_);
  str_tmp += WriteScalar(angular_acceleration_);

  return str_tmp;
}