/**
 * @file InitMagTorquer.cpp
 * @brief Initialize functions for magnetorquer
 */
#include "InitMagTorquer.hpp"

#include "Interface/InitInput/IniAccess.h"

MagTorquer InitMagTorquer(ClockGenerator* clock_gen, int actuator_id, const std::string fname, double compo_step_time,
                          const MagEnvironment* mag_env) {
  IniAccess magtorquer_conf(fname);
  char MTSection[30] = "MAGTORQUER";

  int prescaler = magtorquer_conf.ReadInt(MTSection, "prescaler");
  if (prescaler <= 1) prescaler = 1;

  Vector<kMtqDim * kMtqDim> sf_vec;
  magtorquer_conf.ReadVector(MTSection, "scale_factor_c", sf_vec);
  Matrix<kMtqDim, kMtqDim> scale_factor;
  for (size_t i = 0; i < kMtqDim; i++) {
    for (size_t j = 0; j < kMtqDim; j++) {
      scale_factor[i][j] = sf_vec[i * kMtqDim + j];
    }
  }

  Quaternion q_b2c;
  magtorquer_conf.ReadQuaternion(MTSection, "quaternion_b2c", q_b2c);

  Vector<kMtqDim> max_c;
  magtorquer_conf.ReadVector(MTSection, "max_output_magnetic_moment_c_Am2", max_c);

  Vector<kMtqDim> min_c;
  magtorquer_conf.ReadVector(MTSection, "min_output_magnetic_moment_c_Am2", min_c);

  Vector<kMtqDim> bias_c;
  magtorquer_conf.ReadVector(MTSection, "constant_bias_noise_c_Am2", bias_c);

  double rw_stepwidth = compo_step_time * (double)prescaler;
  Vector<kMtqDim> rw_stddev_c;
  magtorquer_conf.ReadVector(MTSection, "random_walk_standard_deviation_c_Am2", rw_stddev_c);
  Vector<kMtqDim> rw_limit_c;
  magtorquer_conf.ReadVector(MTSection, "random_walk_limit_c_Am2", rw_limit_c);
  Vector<kMtqDim> nr_stddev_c;
  magtorquer_conf.ReadVector(MTSection, "white_noise_standard_deviation_c_Am2", nr_stddev_c);

  MagTorquer magtorquer(prescaler, clock_gen, actuator_id, q_b2c, scale_factor, max_c, min_c, bias_c, rw_stepwidth, rw_stddev_c, rw_limit_c,
                        nr_stddev_c, mag_env);
  return magtorquer;
}

MagTorquer InitMagTorquer(ClockGenerator* clock_gen, PowerPort* power_port, int actuator_id, const std::string fname, double compo_step_time,
                          const MagEnvironment* mag_env) {
  IniAccess magtorquer_conf(fname);
  char MTSection[30] = "MAGTORQUER";

  int prescaler = magtorquer_conf.ReadInt(MTSection, "prescaler");
  if (prescaler <= 1) prescaler = 1;

  Vector<kMtqDim * kMtqDim> sf_vec;
  magtorquer_conf.ReadVector(MTSection, "scale_factor_c", sf_vec);
  Matrix<kMtqDim, kMtqDim> scale_factor;
  for (size_t i = 0; i < kMtqDim; i++) {
    for (size_t j = 0; j < kMtqDim; j++) {
      scale_factor[i][j] = sf_vec[i * kMtqDim + j];
    }
  }

  Quaternion q_b2c;
  magtorquer_conf.ReadQuaternion(MTSection, "quaternion_b2c", q_b2c);

  Vector<kMtqDim> max_c;
  magtorquer_conf.ReadVector(MTSection, "max_output_magnetic_moment_c_Am2", max_c);

  Vector<kMtqDim> min_c;
  magtorquer_conf.ReadVector(MTSection, "min_output_magnetic_moment_c_Am2", min_c);

  Vector<kMtqDim> bias_c;
  magtorquer_conf.ReadVector(MTSection, "constant_bias_noise_c_Am2", bias_c);

  double rw_stepwidth = compo_step_time * (double)prescaler;
  Vector<kMtqDim> rw_stddev_c;
  magtorquer_conf.ReadVector(MTSection, "random_walk_standard_deviation_c_Am2", rw_stddev_c);
  Vector<kMtqDim> rw_limit_c;
  magtorquer_conf.ReadVector(MTSection, "random_walk_limit_c_Am2", rw_limit_c);
  Vector<kMtqDim> nr_stddev_c;
  magtorquer_conf.ReadVector(MTSection, "white_noise_standard_deviation_c_Am2", nr_stddev_c);

  // PowerPort
  power_port->InitializeWithInitializeFile(fname);

  MagTorquer magtorquer(prescaler, clock_gen, power_port, actuator_id, q_b2c, scale_factor, max_c, min_c, bias_c, rw_stepwidth, rw_stddev_c,
                        rw_limit_c, nr_stddev_c, mag_env);
  return magtorquer;
}
