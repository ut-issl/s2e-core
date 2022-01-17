#include "../../../Component/AOCS/MagSensor.h"
#include "../Initialize.h"

MagSensor InitMagSensor(ClockGenerator *clock_gen, int sensor_id,
                        const std::string fname, double compo_step_time,
                        const MagEnvironment *magnet) {
  IniAccess magsensor_conf(fname);
  char MSSection[30] = "MAGSENSOR";

  int prescaler = magsensor_conf.ReadInt(MSSection, "prescaler");
  if (prescaler <= 1)
    prescaler = 1;

  Quaternion q_b2c;
  magsensor_conf.ReadQuaternion(MSSection, "q_b2c", q_b2c);

  // SensorBase
  Vector<kMagDim * kMagDim> sf_vec;
  magsensor_conf.ReadVector(MSSection, "ScaleFactor", sf_vec);
  Matrix<kMagDim, kMagDim> scale_factor;
  for (size_t i = 0; i < kMagDim; i++) {
    for (size_t j = 0; j < kMagDim; j++) {
      scale_factor[i][j] = sf_vec[i * kMagDim + j];
    }
  }
  double range_to_const =
      magsensor_conf.ReadDouble(MSSection, "Range_to_const");
  Vector<kMagDim> range_to_const_c{range_to_const};
  double range_to_zero = magsensor_conf.ReadDouble(MSSection, "Range_to_zero");
  Vector<kMagDim> range_to_zero_c{range_to_zero};

  Vector<kMagDim> bias_c;
  magsensor_conf.ReadVector(MSSection, "Bias_c", bias_c);
  double rw_stepwidth = compo_step_time * (double)prescaler;
  Vector<kMagDim> rw_stddev_c;
  magsensor_conf.ReadVector(MSSection, "rw_stddev_c", rw_stddev_c);
  Vector<kMagDim> rw_limit_c;
  magsensor_conf.ReadVector(MSSection, "rw_limit_c", rw_limit_c);
  Vector<kMagDim> nr_stddev_c;
  magsensor_conf.ReadVector(MSSection, "nr_stddev_c", nr_stddev_c);

  SensorBase<kMagDim> mag_sb(scale_factor, range_to_const_c, range_to_zero_c,
                             bias_c, nr_stddev_c, rw_stepwidth, rw_stddev_c,
                             rw_limit_c);

  MagSensor magsensor(prescaler, clock_gen, mag_sb, sensor_id, q_b2c, magnet);
  return magsensor;
}

MagSensor InitMagSensor(ClockGenerator *clock_gen, PowerPort *power_port,
                        int sensor_id, const std::string fname,
                        double compo_step_time, const MagEnvironment *magnet) {
  IniAccess magsensor_conf(fname);
  char MSSection[30] = "MAGSENSOR";

  int prescaler = magsensor_conf.ReadInt(MSSection, "prescaler");
  if (prescaler <= 1)
    prescaler = 1;

  Quaternion q_b2c;
  magsensor_conf.ReadQuaternion(MSSection, "q_b2c", q_b2c);

  // SensorBase
  Vector<kMagDim * kMagDim> sf_vec;
  magsensor_conf.ReadVector(MSSection, "ScaleFactor", sf_vec);
  Matrix<kMagDim, kMagDim> scale_factor;
  for (size_t i = 0; i < kMagDim; i++) {
    for (size_t j = 0; j < kMagDim; j++) {
      scale_factor[i][j] = sf_vec[i * kMagDim + j];
    }
  }
  double range_to_const =
      magsensor_conf.ReadDouble(MSSection, "Range_to_const");
  Vector<kMagDim> range_to_const_c{range_to_const};
  double range_to_zero = magsensor_conf.ReadDouble(MSSection, "Range_to_zero");
  Vector<kMagDim> range_to_zero_c{range_to_zero};

  Vector<kMagDim> bias_c;
  magsensor_conf.ReadVector(MSSection, "Bias_c", bias_c);
  double rw_stepwidth = compo_step_time * (double)prescaler;
  Vector<kMagDim> rw_stddev_c;
  magsensor_conf.ReadVector(MSSection, "rw_stddev_c", rw_stddev_c);
  Vector<kMagDim> rw_limit_c;
  magsensor_conf.ReadVector(MSSection, "rw_limit_c", rw_limit_c);
  Vector<kMagDim> nr_stddev_c;
  magsensor_conf.ReadVector(MSSection, "nr_stddev_c", nr_stddev_c);

  SensorBase<kMagDim> mag_sb(scale_factor, range_to_const_c, range_to_zero_c,
                             bias_c, nr_stddev_c, rw_stepwidth, rw_stddev_c,
                             rw_limit_c);

  // PowerPort
  double minimum_voltage =
      magsensor_conf.ReadDouble(MSSection, "minimum_voltage");
  power_port->SetMinimumVoltage(minimum_voltage);
  double assumed_power_consumption =
      magsensor_conf.ReadDouble(MSSection, "assumed_power_consumption");
  power_port->AddAssumedPowerConsumption(assumed_power_consumption);

  MagSensor magsensor(prescaler, clock_gen, power_port, mag_sb, sensor_id,
                      q_b2c, magnet);
  return magsensor;
}
