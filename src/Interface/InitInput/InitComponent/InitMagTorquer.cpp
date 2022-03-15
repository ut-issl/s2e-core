#include <Component/AOCS/MagTorquer.h>

#include "../Initialize.h"

MagTorquer InitMagTorquer(ClockGenerator* clock_gen, int actuator_id,
                          const std::string fname, double compo_step_time,
                          const MagEnvironment* mag_env) {
  IniAccess magtorquer_conf(fname);
  char MTSection[30] = "MAGTORQUER";

  int prescaler = magtorquer_conf.ReadInt(MTSection, "prescaler");
  if (prescaler <= 1) prescaler = 1;

  Vector<kMtqDim * kMtqDim> sf_vec;
  magtorquer_conf.ReadVector(MTSection, "ScaleFactor", sf_vec);
  Matrix<kMtqDim, kMtqDim> scale_factor;
  for (size_t i = 0; i < kMtqDim; i++) {
    for (size_t j = 0; j < kMtqDim; j++) {
      scale_factor[i][j] = sf_vec[i * kMtqDim + j];
    }
  }

  Quaternion q_b2c;
  magtorquer_conf.ReadQuaternion(MTSection, "q_b2c", q_b2c);

  Vector<kMtqDim> max_c;
  magtorquer_conf.ReadVector(MTSection, "Max_c", max_c);

  Vector<kMtqDim> min_c;
  magtorquer_conf.ReadVector(MTSection, "Min_c", min_c);

  Vector<kMtqDim> bias_c;
  magtorquer_conf.ReadVector(MTSection, "Bias_c", bias_c);

  double rw_stepwidth = compo_step_time * (double)prescaler;
  Vector<kMtqDim> rw_stddev_c;
  magtorquer_conf.ReadVector(MTSection, "rw_stddev_c", rw_stddev_c);
  Vector<kMtqDim> rw_limit_c;
  magtorquer_conf.ReadVector(MTSection, "rw_limit_c", rw_limit_c);
  Vector<kMtqDim> nr_stddev_c;
  magtorquer_conf.ReadVector(MTSection, "nr_stddev_c", nr_stddev_c);

  MagTorquer magtorquer(prescaler, clock_gen, actuator_id, q_b2c, scale_factor,
                        max_c, min_c, bias_c, rw_stepwidth, rw_stddev_c,
                        rw_limit_c, nr_stddev_c, mag_env);
  return magtorquer;
}

MagTorquer InitMagTorquer(ClockGenerator* clock_gen, PowerPort* power_port,
                          int actuator_id, const std::string fname,
                          double compo_step_time,
                          const MagEnvironment* mag_env) {
  IniAccess magtorquer_conf(fname);
  char MTSection[30] = "MAGTORQUER";

  int prescaler = magtorquer_conf.ReadInt(MTSection, "prescaler");
  if (prescaler <= 1) prescaler = 1;

  Vector<kMtqDim * kMtqDim> sf_vec;
  magtorquer_conf.ReadVector(MTSection, "ScaleFactor", sf_vec);
  Matrix<kMtqDim, kMtqDim> scale_factor;
  for (size_t i = 0; i < kMtqDim; i++) {
    for (size_t j = 0; j < kMtqDim; j++) {
      scale_factor[i][j] = sf_vec[i * kMtqDim + j];
    }
  }

  Quaternion q_b2c;
  magtorquer_conf.ReadQuaternion(MTSection, "q_b2c", q_b2c);

  Vector<kMtqDim> max_c;
  magtorquer_conf.ReadVector(MTSection, "Max_c", max_c);

  Vector<kMtqDim> min_c;
  magtorquer_conf.ReadVector(MTSection, "Min_c", min_c);

  Vector<kMtqDim> bias_c;
  magtorquer_conf.ReadVector(MTSection, "Bias_c", bias_c);

  double rw_stepwidth = compo_step_time * (double)prescaler;
  Vector<kMtqDim> rw_stddev_c;
  magtorquer_conf.ReadVector(MTSection, "rw_stddev_c", rw_stddev_c);
  Vector<kMtqDim> rw_limit_c;
  magtorquer_conf.ReadVector(MTSection, "rw_limit_c", rw_limit_c);
  Vector<kMtqDim> nr_stddev_c;
  magtorquer_conf.ReadVector(MTSection, "nr_stddev_c", nr_stddev_c);

  // PowerPort
  double minimum_voltage =
      magtorquer_conf.ReadDouble(MTSection, "minimum_voltage");
  power_port->SetMinimumVoltage(minimum_voltage);
  double assumed_power_consumption =
      magtorquer_conf.ReadDouble(MTSection, "assumed_power_consumption");
  power_port->SetAssumedPowerConsumption(assumed_power_consumption);

  MagTorquer magtorquer(prescaler, clock_gen, power_port, actuator_id, q_b2c,
                        scale_factor, max_c, min_c, bias_c, rw_stepwidth,
                        rw_stddev_c, rw_limit_c, nr_stddev_c, mag_env);
  return magtorquer;
}
