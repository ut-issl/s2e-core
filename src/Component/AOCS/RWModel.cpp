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
  return rpm * 2.0*M_PI / 60.0;
}

static double angularVelocity2rpm(double angular_velocity)
{
  return angular_velocity * 60 / (2.0 * M_PI);
}

RWModel::RWModel(ClockGenerator* clock_gen,
  double step_width,
	double init_rpm,
	double inertia,
	double max_rpm,
	bool motor_drive_init,
	Vector<3> torque_transition,
	double dead_seconds,
	double coastring_end,
	Vector<3>  ordinary_lag_coef,
	Vector<3> coasting_lag_coef,
	double max_torque,
	double target_angular_velocity) :
	ComponentBase(10,clock_gen),
	kDeadSeconds_(dead_seconds),
	dead_seconds_(dead_seconds),
	inertia_(inertia),
	motor_drive_(motor_drive_init),
	kTorqueTransition_(torque_transition),
	ode_angular_velocity_(step_width,
		rpm2angularVelocity(init_rpm),
		rpm2angularVelocity(init_rpm),
		ordinary_lag_coef),
	angular_velocity_limit_(rpm2angularVelocity(max_rpm)),
	kStepSeconds_(step_width),
	kOrdinaryLagCoef_(ordinary_lag_coef),
	kCoastingLagCoef_(coasting_lag_coef),
	target_angular_accl_before_(0),
	kCoastingEnd_(rpm2angularVelocity(coastring_end)),
	MAX_TORQUE_(max_torque)
{
}


void RWModel::MainRoutine(int count)
{

}

Vector<3> RWModel::CalcTorque(double com_period_ms)
{
	double com_period_ = com_period_ms/1000;
	if (com_period_ < kStepSeconds_)
	{
        ode_angular_velocity_.setStepWidth(com_period_);
	}
	if (!motor_drive_)
	{
		double pre_angular_velocity = ode_angular_velocity_.getAngularVelocity();
		ode_angular_velocity_.setLagCoef(kCoastingLagCoef_);
		ode_angular_velocity_.setTargetAngularVelocity(0);
		for (int i = 0; i < (com_period_ / kStepSeconds_); i++)
		{
			++ode_angular_velocity_; // propagate()
		}
		double angular_velocity = ode_angular_velocity_.getAngularVelocity();
        //std::cout << "angular_vel: "<< angular_velocity<< "\n";
		angular_velocity_rpm = angularVelocity2rpm(ode_angular_velocity_.getAngularVelocity());
		angular_accleration_ = (angular_velocity - pre_angular_velocity) / com_period_;
		rwtorque_b_ = inertia_ * (angular_accleration_)* kTorqueTransition_;
		angular_momentum_b_ = inertia_ * angular_velocity;
		return rwtorque_b_;
	}
	else
	{
		ode_angular_velocity_.setLagCoef(kOrdinaryLagCoef_);
		double pre_angular_velocity = ode_angular_velocity_.getAngularVelocity();
		double angular_accl = targets_angular_accl_.front();
		double target_angular_velocity = pre_angular_velocity + angular_accl * com_period_;
		targets_angular_accl_.erase(targets_angular_accl_.begin());
		if (targets_angular_accl_.size() == 0)
		{
			//error handling(avoiding segmentation fault)
			targets_angular_accl_.push_back(0);
		}
		ode_angular_velocity_.setTargetAngularVelocity(target_angular_velocity);
		for (int i = 0; i < (com_period_ / kStepSeconds_); i++)//通信速度分の伝搬を行う。
		{
			++ode_angular_velocity_;//propagate
		}
		double angular_velocity = ode_angular_velocity_.getAngularVelocity();
		angular_velocity_rpm = angularVelocity2rpm(ode_angular_velocity_.getAngularVelocity());
		angular_accleration_ = (angular_velocity - pre_angular_velocity) / com_period_;
        rwtorque_b_ = inertia_ * (angular_accleration_)*kTorqueTransition_;
        angular_momentum_b_ = inertia_ * angular_velocity;// *kTorqueTransition_;
		return rwtorque_b_;
	}
}

Vector<3> RWModel::GetTorque()
{
  return rwtorque_b_;
}


bool RWModel::isMotorDrived()
{
  return motor_drive_;
}

double RWModel::GetAngularVelocity()
{
  return ode_angular_velocity_.getAngularVelocity();
}

double RWModel::GetAngularMomentLimit(){
    return kCoastingEnd_*inertia_;
}

double RWModel::GetRPM()
{
	return angularVelocity2rpm(ode_angular_velocity_.getAngularVelocity());
}

double RWModel::GetAngularMomentum()
{
  return angular_momentum_b_;
}

void RWModel::SetTorque(double torque, double ctrl_cycle)
{
	//目標トルクからRW目標角加速度履歴を追加
	double angular_acceleration;
	ctrl_cycle /= 1000;
	int sign;
	torque > 0 ? sign = 1 : sign = -1;
	if (abs(torque) < MAX_TORQUE_)
	{
		angular_acceleration = torque / inertia_; // 単位はrad/sec
	}
	else
	{
		angular_acceleration = sign * MAX_TORQUE_ / inertia_;
	}
	if (abs(target_angular_accl_before_) < std::numeric_limits<double>::epsilon() && abs(angular_acceleration) > std::numeric_limits<double>::epsilon())
	{
		//無駄時間追加処理
		for (int i = 1; i <= (kDeadSeconds_ / ctrl_cycle); i++)
		{
			targets_angular_accl_.push_back(0);
		}
		targets_angular_accl_.push_back(angular_acceleration);
	}
	else
	{
		targets_angular_accl_.push_back(angular_acceleration);
	}
	target_angular_accl_before_ = angular_acceleration;
}

void RWModel::SetLimits(double angular_velocity_limit)
{
	angular_velocity_limit_ = rpm2angularVelocity(angular_velocity_limit);
}

void RWModel::SetDrive(bool flag)
{
	motor_drive_ = flag;
}

string RWModel::GetLogHeader() const
{
  string str_tmp = "";

  str_tmp += WriteScalar("angular_velocity", "rad/s");
  str_tmp += WriteScalar("angular_velocity_upperlimit", "rad/s");
  str_tmp += WriteScalar("angular_accelaration", "rad/s^2");
  str_tmp += WriteScalar("angular_velocity_rpm", "rpm");
  str_tmp += WriteScalar("angular_momentum_b_", "Nms");

  return str_tmp;
}

string RWModel::GetLogValue() const
{
  string str_tmp = "";

  str_tmp += WriteScalar(ode_angular_velocity_.getAngularVelocity());
  str_tmp += WriteScalar(angular_velocity_limit_);
  str_tmp += WriteScalar(angular_accleration_);
  str_tmp += WriteScalar(angular_velocity_rpm);
  str_tmp += WriteScalar(angular_momentum_b_);
  return str_tmp;
}