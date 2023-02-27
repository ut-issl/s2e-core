/**
 * @file magnetorquer.cpp
 * @brief Class to emulate magnetorquer
 */

#include "magnetorquer.hpp"

#include <library/logger/logger.hpp>
#include <library/math/matrix_vector.hpp>
#include <library/math/quaternion.hpp>
#include <library/randomization/global_randomization.hpp>

MagTorquer::MagTorquer(const int prescaler, ClockGenerator* clock_generator, const int id, const Quaternion& quaternion_b2c,
                       const libra::Matrix<kMtqDim, kMtqDim>& scale_factor, const libra::Vector<kMtqDim>& max_c, const libra::Vector<kMtqDim>& min_c,
                       const libra::Vector<kMtqDim>& bias_c, double rw_stepwidth, const libra::Vector<kMtqDim>& rw_stddev_c,
                       const libra::Vector<kMtqDim>& rw_limit_c, const libra::Vector<kMtqDim>& nr_stddev_c, const GeomagneticField* mag_env)
    : Component(prescaler, clock_generator),
      component_id_(id),
      quaternion_b2c_(quaternion_b2c),
      q_c2b_(quaternion_b2c_.Conjugate()),
      scale_factor_(scale_factor),
      max_c_(max_c),
      min_c_(min_c),
      bias_c_(bias_c),
      n_rw_c_(rw_stepwidth, rw_stddev_c, rw_limit_c),
      mag_env_(mag_env) {
  for (size_t i = 0; i < kMtqDim; i++) {
    nrs_c_[i].SetParameters(0.0, nr_stddev_c[i]);  // global_randomization.MakeSeed()
  }
}

MagTorquer::MagTorquer(const int prescaler, ClockGenerator* clock_generator, PowerPort* power_port, const int id, const Quaternion& quaternion_b2c,
                       const libra::Matrix<kMtqDim, kMtqDim>& scale_factor, const libra::Vector<kMtqDim>& max_c, const libra::Vector<kMtqDim>& min_c,
                       const libra::Vector<kMtqDim>& bias_c, double rw_stepwidth, const libra::Vector<kMtqDim>& rw_stddev_c,
                       const libra::Vector<kMtqDim>& rw_limit_c, const libra::Vector<kMtqDim>& nr_stddev_c, const GeomagneticField* mag_env)
    : Component(prescaler, clock_generator, power_port),
      component_id_(id),
      quaternion_b2c_(quaternion_b2c),
      q_c2b_(quaternion_b2c_.Conjugate()),
      scale_factor_(scale_factor),
      max_c_(max_c),
      min_c_(min_c),
      bias_c_(bias_c),
      n_rw_c_(rw_stepwidth, rw_stddev_c, rw_limit_c),
      mag_env_(mag_env) {
  for (size_t i = 0; i < kMtqDim; i++) {
    nrs_c_[i].SetParameters(0.0, nr_stddev_c[i]);  // global_randomization.MakeSeed()
  }
}

void MagTorquer::MainRoutine(int count) {
  UNUSED(count);

  CalcOutputTorque();
}

void MagTorquer::PowerOffRoutine() { torque_b_ *= 0.0; }

libra::Vector<kMtqDim> MagTorquer::CalcOutputTorque(void) {
  for (size_t i = 0; i < kMtqDim; ++i) {
    // Limit Check
    if (mag_moment_c_[i] > max_c_[i]) {
      mag_moment_c_[i] = max_c_[i];
    } else if (mag_moment_c_[i] < min_c_[i]) {
      mag_moment_c_[i] = min_c_[i];
    }
    // Add noise
    mag_moment_c_[i] += bias_c_[i];
    mag_moment_c_[i] += n_rw_c_[i];
    mag_moment_c_[i] += nrs_c_[i];
  }
  mag_moment_c_ = scale_factor_ * mag_moment_c_;

  // Frame conversion component to body
  mag_moment_b_ = q_c2b_.FrameConversion(mag_moment_c_);
  // Calc magnetic torque [Nm]
  torque_b_ = OuterProduct(mag_moment_b_, knT2T * mag_env_->GetGeomagneticField_b_nT());
  // Update Random Walk
  ++n_rw_c_;

  return torque_b_;
}

std::string MagTorquer::GetLogHeader() const {
  std::string str_tmp = "";
  const std::string actuator_id = std::to_string(static_cast<long long>(component_id_));
  std::string actuator_name = "magnetorquer" + actuator_id + "_";

  str_tmp += WriteVector(actuator_name + "output_magnetic_moment", "b", "Am2", kMtqDim);
  str_tmp += WriteVector(actuator_name + "output_torque", "b", "Nm", kMtqDim);

  return str_tmp;
}

std::string MagTorquer::GetLogValue() const {
  std::string str_tmp = "";
  str_tmp += WriteVector(mag_moment_b_);
  str_tmp += WriteVector(torque_b_);

  return str_tmp;
}
