/**
 * @file magnetorquer.cpp
 * @brief Class to emulate magnetorquer
 */

#include "magnetorquer.hpp"

#include <logger/logger.hpp>
#include <math_physics/math/matrix_vector.hpp>
#include <math_physics/math/quaternion.hpp>
#include <math_physics/randomization/global_randomization.hpp>
#include <setting_file_reader/initialize_file_access.hpp>

Magnetorquer::Magnetorquer(const int prescaler, ClockGenerator* clock_generator, const int component_id, const libra::Quaternion& quaternion_b2c,
                           const math::Matrix<kMtqDimension, kMtqDimension>& scale_factor,
                           const math::Vector<kMtqDimension>& max_magnetic_moment_c_Am2,
                           const math::Vector<kMtqDimension>& min_magnetic_moment_c_Am2, const math::Vector<kMtqDimension>& bias_noise_c_Am2_,
                           double random_walk_step_width_s, const math::Vector<kMtqDimension>& random_walk_standard_deviation_c_Am2,
                           const math::Vector<kMtqDimension>& random_walk_limit_c_Am2,
                           const math::Vector<kMtqDimension>& normal_random_standard_deviation_c_Am2, const GeomagneticField* geomagnetic_field)
    : Component(prescaler, clock_generator),
      component_id_(component_id),
      quaternion_b2c_(quaternion_b2c),
      quaternion_c2b_(quaternion_b2c_.Conjugate()),
      scale_factor_(scale_factor),
      max_magnetic_moment_c_Am2_(max_magnetic_moment_c_Am2),
      min_magnetic_moment_c_Am2_(min_magnetic_moment_c_Am2),
      bias_noise_c_Am2_(bias_noise_c_Am2_),
      random_walk_c_Am2_(random_walk_step_width_s, random_walk_standard_deviation_c_Am2, random_walk_limit_c_Am2),
      geomagnetic_field_(geomagnetic_field) {
  for (size_t i = 0; i < kMtqDimension; i++) {
    random_noise_c_Am2_[i].SetParameters(0.0, normal_random_standard_deviation_c_Am2[i]);  // global_randomization.MakeSeed()
  }
}

Magnetorquer::Magnetorquer(const int prescaler, ClockGenerator* clock_generator, PowerPort* power_port, const int component_id,
                           const libra::Quaternion& quaternion_b2c, const math::Matrix<kMtqDimension, kMtqDimension>& scale_factor,
                           const math::Vector<kMtqDimension>& max_magnetic_moment_c_Am2,
                           const math::Vector<kMtqDimension>& min_magnetic_moment_c_Am2, const math::Vector<kMtqDimension>& bias_noise_c_Am2_,
                           double random_walk_step_width_s, const math::Vector<kMtqDimension>& random_walk_standard_deviation_c_Am2,
                           const math::Vector<kMtqDimension>& random_walk_limit_c_Am2,
                           const math::Vector<kMtqDimension>& normal_random_standard_deviation_c_Am2, const GeomagneticField* geomagnetic_field)
    : Component(prescaler, clock_generator, power_port),
      component_id_(component_id),
      quaternion_b2c_(quaternion_b2c),
      quaternion_c2b_(quaternion_b2c_.Conjugate()),
      scale_factor_(scale_factor),
      max_magnetic_moment_c_Am2_(max_magnetic_moment_c_Am2),
      min_magnetic_moment_c_Am2_(min_magnetic_moment_c_Am2),
      bias_noise_c_Am2_(bias_noise_c_Am2_),
      random_walk_c_Am2_(random_walk_step_width_s, random_walk_standard_deviation_c_Am2, random_walk_limit_c_Am2),
      geomagnetic_field_(geomagnetic_field) {
  for (size_t i = 0; i < kMtqDimension; i++) {
    random_noise_c_Am2_[i].SetParameters(0.0, normal_random_standard_deviation_c_Am2[i]);  // global_randomization.MakeSeed()
  }
}

void Magnetorquer::MainRoutine(const int time_count) {
  UNUSED(time_count);

  CalcOutputTorque();
}

void Magnetorquer::PowerOffRoutine() {
  torque_b_Nm_ *= 0.0;
  output_magnetic_moment_c_Am2_ *= 0.0;
  output_magnetic_moment_b_Am2_ *= 0.0;
}

math::Vector<kMtqDimension> Magnetorquer::CalcOutputTorque(void) {
  for (size_t i = 0; i < kMtqDimension; ++i) {
    // Limit Check
    if (output_magnetic_moment_c_Am2_[i] > max_magnetic_moment_c_Am2_[i]) {
      output_magnetic_moment_c_Am2_[i] = max_magnetic_moment_c_Am2_[i];
    } else if (output_magnetic_moment_c_Am2_[i] < min_magnetic_moment_c_Am2_[i]) {
      output_magnetic_moment_c_Am2_[i] = min_magnetic_moment_c_Am2_[i];
    }
    // Add noise
    output_magnetic_moment_c_Am2_[i] += bias_noise_c_Am2_[i];
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

std::string Magnetorquer::GetLogHeader() const {
  std::string str_tmp = "";
  const std::string actuator_id = std::to_string(static_cast<long long>(component_id_));
  std::string actuator_name = "magnetorquer" + actuator_id + "_";

  str_tmp += WriteVector(actuator_name + "output_magnetic_moment", "b", "Am2", kMtqDimension);
  str_tmp += WriteVector(actuator_name + "output_torque", "b", "Nm", kMtqDimension);

  return str_tmp;
}

std::string Magnetorquer::GetLogValue() const {
  std::string str_tmp = "";
  str_tmp += WriteVector(output_magnetic_moment_b_Am2_);
  str_tmp += WriteVector(torque_b_Nm_);

  return str_tmp;
}

Magnetorquer InitMagnetorquer(ClockGenerator* clock_generator, int actuator_id, const std::string file_name, double component_step_time_s,
                              const GeomagneticField* geomagnetic_field) {
  IniAccess magtorquer_conf(file_name);
  const char* sensor_name = "MAGNETORQUER_";
  const std::string section_name = sensor_name + std::to_string(static_cast<long long>(actuator_id));
  const char* MTSection = section_name.c_str();

  int prescaler = magtorquer_conf.ReadInt(MTSection, "prescaler");
  if (prescaler <= 1) prescaler = 1;

  math::Vector<kMtqDimension * kMtqDimension> sf_vec;
  magtorquer_conf.ReadVector(MTSection, "scale_factor_c", sf_vec);
  math::Matrix<kMtqDimension, kMtqDimension> scale_factor;
  for (size_t i = 0; i < kMtqDimension; i++) {
    for (size_t j = 0; j < kMtqDimension; j++) {
      scale_factor[i][j] = sf_vec[i * kMtqDimension + j];
    }
  }

  libra::Quaternion quaternion_b2c;
  magtorquer_conf.ReadQuaternion(MTSection, "quaternion_b2c", quaternion_b2c);

  math::Vector<kMtqDimension> max_magnetic_moment_c_Am2;
  magtorquer_conf.ReadVector(MTSection, "max_output_magnetic_moment_c_Am2", max_magnetic_moment_c_Am2);

  math::Vector<kMtqDimension> min_magnetic_moment_c_Am2;
  magtorquer_conf.ReadVector(MTSection, "min_output_magnetic_moment_c_Am2", min_magnetic_moment_c_Am2);

  math::Vector<kMtqDimension> bias_noise_c_Am2;
  magtorquer_conf.ReadVector(MTSection, "constant_bias_noise_c_Am2", bias_noise_c_Am2);

  double random_walk_step_width_s = component_step_time_s * (double)prescaler;
  math::Vector<kMtqDimension> random_walk_standard_deviation_c_Am2;
  magtorquer_conf.ReadVector(MTSection, "random_walk_standard_deviation_c_Am2", random_walk_standard_deviation_c_Am2);
  math::Vector<kMtqDimension> random_walk_limit_c_Am2;
  magtorquer_conf.ReadVector(MTSection, "random_walk_limit_c_Am2", random_walk_limit_c_Am2);
  math::Vector<kMtqDimension> normal_random_standard_deviation_c_Am2;
  magtorquer_conf.ReadVector(MTSection, "white_noise_standard_deviation_c_Am2", normal_random_standard_deviation_c_Am2);

  Magnetorquer magtorquer(prescaler, clock_generator, actuator_id, quaternion_b2c, scale_factor, max_magnetic_moment_c_Am2, min_magnetic_moment_c_Am2,
                          bias_noise_c_Am2, random_walk_step_width_s, random_walk_standard_deviation_c_Am2, random_walk_limit_c_Am2,
                          normal_random_standard_deviation_c_Am2, geomagnetic_field);
  return magtorquer;
}

Magnetorquer InitMagnetorquer(ClockGenerator* clock_generator, PowerPort* power_port, int actuator_id, const std::string file_name,
                              double component_step_time_s, const GeomagneticField* geomagnetic_field) {
  IniAccess magtorquer_conf(file_name);
  const char* sensor_name = "MAGNETORQUER_";
  const std::string section_name = sensor_name + std::to_string(static_cast<long long>(actuator_id));
  const char* MTSection = section_name.c_str();

  int prescaler = magtorquer_conf.ReadInt(MTSection, "prescaler");
  if (prescaler <= 1) prescaler = 1;

  math::Vector<kMtqDimension * kMtqDimension> sf_vec;
  magtorquer_conf.ReadVector(MTSection, "scale_factor_c", sf_vec);
  math::Matrix<kMtqDimension, kMtqDimension> scale_factor;
  for (size_t i = 0; i < kMtqDimension; i++) {
    for (size_t j = 0; j < kMtqDimension; j++) {
      scale_factor[i][j] = sf_vec[i * kMtqDimension + j];
    }
  }

  libra::Quaternion quaternion_b2c;
  magtorquer_conf.ReadQuaternion(MTSection, "quaternion_b2c", quaternion_b2c);

  math::Vector<kMtqDimension> max_magnetic_moment_c_Am2;
  magtorquer_conf.ReadVector(MTSection, "max_output_magnetic_moment_c_Am2", max_magnetic_moment_c_Am2);

  math::Vector<kMtqDimension> min_magnetic_moment_c_Am2;
  magtorquer_conf.ReadVector(MTSection, "min_output_magnetic_moment_c_Am2", min_magnetic_moment_c_Am2);

  math::Vector<kMtqDimension> bias_noise_c_Am2;
  magtorquer_conf.ReadVector(MTSection, "constant_bias_noise_c_Am2", bias_noise_c_Am2);

  double random_walk_step_width_s = component_step_time_s * (double)prescaler;
  math::Vector<kMtqDimension> random_walk_standard_deviation_c_Am2;
  magtorquer_conf.ReadVector(MTSection, "random_walk_standard_deviation_c_Am2", random_walk_standard_deviation_c_Am2);
  math::Vector<kMtqDimension> random_walk_limit_c_Am2;
  magtorquer_conf.ReadVector(MTSection, "random_walk_limit_c_Am2", random_walk_limit_c_Am2);
  math::Vector<kMtqDimension> normal_random_standard_deviation_c_Am2;
  magtorquer_conf.ReadVector(MTSection, "white_noise_standard_deviation_c_Am2", normal_random_standard_deviation_c_Am2);

  // PowerPort
  power_port->InitializeWithInitializeFile(file_name);

  Magnetorquer magtorquer(prescaler, clock_generator, power_port, actuator_id, quaternion_b2c, scale_factor, max_magnetic_moment_c_Am2,
                          min_magnetic_moment_c_Am2, bias_noise_c_Am2, random_walk_step_width_s, random_walk_standard_deviation_c_Am2,
                          random_walk_limit_c_Am2, normal_random_standard_deviation_c_Am2, geomagnetic_field);
  return magtorquer;
}
