#include "MagTorquer.h"
#include "../../Library/math/GlobalRand.h"
#include "../../Library/math/Quaternion.hpp"
#include "../../Library/math/MatVec.hpp"
#include "../../Interface/LogOutput/Logger.h"

MagTorquer::MagTorquer(
  const int prescaler, 
  ClockGenerator* clock_gen,
  const int id,
  const Quaternion& q_b2c,
  const libra::Matrix<kMtqDim, kMtqDim>& scale_factor,
  const libra::Vector<kMtqDim>& max_c,
  const libra::Vector<kMtqDim>& min_c,
  const libra::Vector<kMtqDim>& bias_c,
  double rw_stepwidth,
  const libra::Vector<kMtqDim>& rw_stddev_c,
  const libra::Vector<kMtqDim>& rw_limit_c,
  const libra::Vector<kMtqDim>& nr_stddev_c,
  const MagEnvironment *mag_env)
  : ComponentBase(prescaler,clock_gen), id_(id),
  q_b2c_(q_b2c), q_c2b_(q_b2c_.conjugate()), scale_factor_(scale_factor), 
  max_c_(max_c), min_c_(min_c), bias_c_(bias_c), 
  n_rw_c_(rw_stepwidth, rw_stddev_c, rw_limit_c),
  mag_env_(mag_env)
{
  for(size_t i=0;i<kMtqDim;i++)
  {
    nrs_c_[i].set_param(0.0, nr_stddev_c[i]);  //g_rand.MakeSeed()
  }
}

MagTorquer::MagTorquer(
  const int prescaler, 
  ClockGenerator* clock_gen,
  PowerPort* power_port,
  const int id,
  const Quaternion& q_b2c,
  const libra::Matrix<kMtqDim, kMtqDim>& scale_factor,
  const libra::Vector<kMtqDim>& max_c,
  const libra::Vector<kMtqDim>& min_c,
  const libra::Vector<kMtqDim>& bias_c,
  double rw_stepwidth,
  const libra::Vector<kMtqDim>& rw_stddev_c,
  const libra::Vector<kMtqDim>& rw_limit_c,
  const libra::Vector<kMtqDim>& nr_stddev_c,
  const MagEnvironment *mag_env)
  : ComponentBase(prescaler,clock_gen,power_port), id_(id),
  q_b2c_(q_b2c), q_c2b_(q_b2c_.conjugate()), scale_factor_(scale_factor), 
  max_c_(max_c), min_c_(min_c), bias_c_(bias_c), 
  n_rw_c_(rw_stepwidth, rw_stddev_c, rw_limit_c),
  mag_env_(mag_env)
{
  for(size_t i=0;i<kMtqDim;i++)
  {
    nrs_c_[i].set_param(0.0, nr_stddev_c[i]);  //g_rand.MakeSeed()
  }
}

void MagTorquer::MainRoutine(int count)
{
  CalcOutputTorque();
}

libra::Vector<kMtqDim> MagTorquer::CalcOutputTorque(void)
{
  for (size_t i = 0; i < kMtqDim; ++i)
  {
    // Limit Check
    if (mag_moment_c_[i] > max_c_[i])
    {
      mag_moment_c_[i] = max_c_[i];
    }
    else if (mag_moment_c_[i] < min_c_[i])
    {
      mag_moment_c_[i] = min_c_[i];
    }
    // Add noise
    mag_moment_c_[i] += bias_c_[i];
    mag_moment_c_[i] += n_rw_c_[i];
    mag_moment_c_[i] += nrs_c_[i];
  }
  mag_moment_c_ = scale_factor_ * mag_moment_c_;

  // Frame conversion component to body
  mag_moment_b_ = q_c2b_.frame_conv(mag_moment_c_);
  // Calc magnetic torque [Nm]
  torque_b_ = outer_product(mag_moment_b_, knT2T * mag_env_->GetMag_b());
  // Update Random Walk
  ++n_rw_c_;

  return torque_b_;
}

std::string MagTorquer::GetLogHeader() const
{
  std::string str_tmp = "";
  const std::string st_sensor_id = std::to_string(static_cast<long long>(id_));
  const char *cs = st_sensor_id.data();
  std::string MSSection = "mag_torquer";

  str_tmp += WriteVector(MSSection + cs, "b", "Am^2", kMtqDim);
  str_tmp += WriteVector(MSSection + cs, "b", "Nm", kMtqDim);

  return str_tmp;
}

std::string MagTorquer::GetLogValue() const
{
  std::string str_tmp = "";
  str_tmp += WriteVector(mag_moment_b_);
  str_tmp += WriteVector(torque_b_);

  return str_tmp;
}
