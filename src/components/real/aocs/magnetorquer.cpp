/**
 * @file magnetorquer.cpp
 * @brief Class to emulate magnetorquer
 */

#include "magnetorquer.hpp"

#include <library/logger/logger.hpp>
#include <library/math/matrix_vector.hpp>
#include <library/math/quaternion.hpp>
#include <library/randomization/global_randomization.hpp>

MagTorquer::MagTorquer(const int prescaler, ClockGenerator* clock_generator, const int component_id, const Quaternion& quaternion_b2c,
                       const libra::Matrix<kMtqDim, kMtqDim>& scale_factor, const libra::Vector<kMtqDim>& max_c, const libra::Vector<kMtqDim>& min_c,
                       const libra::Vector<kMtqDim>& bias_c, double rw_stepwidth, const libra::Vector<kMtqDim>& rw_stddev_c,
                       const libra::Vector<kMtqDim>& rw_limit_c, const libra::Vector<kMtqDim>& nr_stddev_c, const GeomagneticField* mag_env)
    : Component(prescaler, clock_generator),
      component_id_(component_id),
      quaternion_b2c_(quaternion_b2c),
      quaternion_c2b_(quaternion_b2c_.Conjugate()),
      scale_factor_(scale_factor),
      max_magnetic_moment_c_Am2_(max_c),
      min_magnetic_moment_c_Am2_(min_c),
      bias_c_Am2_(bias_c),
      random_walk_c_Am2_(rw_stepwidth, rw_stddev_c, rw_limit_c),
      geomagnetic_field_(mag_env) {
  for (size_t i = 0; i < kMtqDim; i++) {
    random_noise_c_Am2_[i].SetParameters(0.0, nr_stddev_c[i]);  // global_randomization.MakeSeed()
  }
}

MagTorquer::MagTorquer(const int prescaler, ClockGenerator* clock_generator, PowerPort* power_port, const int component_id,
                       const Quaternion& quaternion_b2c, const libra::Matrix<kMtqDim, kMtqDim>& scale_factor, const libra::Vector<kMtqDim>& max_c,
                       const libra::Vector<kMtqDim>& min_c, const libra::Vector<kMtqDim>& bias_c, double rw_stepwidth,
                       const libra::Vector<kMtqDim>& rw_stddev_c, const libra::Vector<kMtqDim>& rw_limit_c, const libra::Vector<kMtqDim>& nr_stddev_c,
                       const GeomagneticField* mag_env)
    : Component(prescaler, clock_generator, power_port),
      component_id_(component_id),
      quaternion_b2c_(quaternion_b2c),
      quaternion_c2b_(quaternion_b2c_.Conjugate()),
      scale_factor_(scale_factor),
      max_magnetic_moment_c_Am2_(max_c),
      min_magnetic_moment_c_Am2_(min_c),
      bias_c_Am2_(bias_c),
      random_walk_c_Am2_(rw_stepwidth, rw_stddev_c, rw_limit_c),
      geomagnetic_field_(mag_env) {
  for (size_t i = 0; i < kMtqDim; i++) {
    random_noise_c_Am2_[i].SetParameters(0.0, nr_stddev_c[i]);  // global_randomization.MakeSeed()
  }
}

void MagTorquer::MainRoutine(int count) {
  UNUSED(count);

  CalcOutputTorque();
}

void MagTorquer::PowerOffRoutine() { torque_b_Nm_ *= 0.0; }

libra::Vector<kMtqDim> MagTorquer::CalcOutputTorque(void) {
  for (size_t i = 0; i < kMtqDim; ++i) {
    // Limit Check
    if (output_magnetic_moment_c_Am2_[i] > max_magnetic_moment_c_Am2_[i]) {
      output_magnetic_moment_c_Am2_[i] = max_magnetic_moment_c_Am2_[i];
    } else if (output_magnetic_moment_c_Am2_[i] < min_magnetic_moment_c_Am2_[i]) {
      output_magnetic_moment_c_Am2_[i] = min_magnetic_moment_c_Am2_[i];
    }
    // Add noise
    output_magnetic_moment_c_Am2_[i] += bias_c_Am2_[i];
    output_magnetic_moment_c_Am2_[i] += random_walk_c_Am2_[i];
    output_magnetic_moment_c_Am2_[i] += random_noise_c_Am2_[i];
  }
  output_magnetic_moment_c_Am2_ = scale_factor_ * output_magnetic_moment_c_Am2_;

  // Frame conversion component to body
  output_magnetic_moment_b_Am2_ = quaternion_c2b_.FrameConversion(output_magnetic_moment_c_Am2_);
  // Calc magnetic torque [Nm]
  torque_b_Nm_ = OuterProduct(output_magnetic_moment_b_Am2_, kConvertNanoT2T * geomagnetic_field_->GetGeomagneticField_b_nT());
  // Update Random Walk
  ++random_walk_c_Am2_;

  return torque_b_Nm_;
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
  str_tmp += WriteVector(output_magnetic_moment_b_Am2_);
  str_tmp += WriteVector(torque_b_Nm_);

  return str_tmp;
}
