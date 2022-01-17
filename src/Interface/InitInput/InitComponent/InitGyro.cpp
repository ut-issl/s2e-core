#include "../../../Component/AOCS/Gyro.h"
#include "../Initialize.h"

Gyro InitGyro(ClockGenerator *clock_gen, int sensor_id, const std::string fname,
              double compo_step_time, const Dynamics *dynamics) {
  IniAccess gyro_conf(fname);
  char GSection[30] = "GYRO";

  Quaternion q_b2c;
  gyro_conf.ReadQuaternion(GSection, "q_b2c", q_b2c);
  int prescaler = gyro_conf.ReadInt(GSection, "prescaler");
  if (prescaler <= 1)
    prescaler = 1;

  // SensorBase
  Vector<kGyroDim * kGyroDim> sf_vec;
  gyro_conf.ReadVector(GSection, "ScaleFactor", sf_vec);
  Matrix<kGyroDim, kGyroDim> scale_factor;
  for (size_t i = 0; i < kGyroDim; i++) {
    for (size_t j = 0; j < kGyroDim; j++) {
      scale_factor[i][j] = sf_vec[i * kGyroDim + j];
    }
  }
  double range_to_const = gyro_conf.ReadDouble(GSection, "Range_to_const");
  Vector<kGyroDim> range_to_const_c{range_to_const};
  double range_to_zero = gyro_conf.ReadDouble(GSection, "Range_to_zero");
  Vector<kGyroDim> range_to_zero_c{range_to_zero};

  Vector<kGyroDim> bias_c;
  gyro_conf.ReadVector(GSection, "Bias_c", bias_c);

  double rw_stepwidth = compo_step_time * (double)prescaler;
  Vector<kGyroDim> rw_stddev_c;
  gyro_conf.ReadVector(GSection, "rw_stddev_c", rw_stddev_c);
  Vector<kGyroDim> rw_limit_c;
  gyro_conf.ReadVector(GSection, "rw_limit_c", rw_limit_c);
  Vector<kGyroDim> nr_stddev_c;
  gyro_conf.ReadVector(GSection, "nr_stddev_c", nr_stddev_c);

  SensorBase<kGyroDim> gyro_sb(scale_factor, range_to_const_c, range_to_zero_c,
                               bias_c, nr_stddev_c, rw_stepwidth, rw_stddev_c,
                               rw_limit_c);

  Gyro gyro(prescaler, clock_gen, gyro_sb, sensor_id, q_b2c, dynamics);

  return gyro;
}

Gyro InitGyro(ClockGenerator *clock_gen, PowerPort *power_port, int sensor_id,
              const std::string fname, double compo_step_time,
              const Dynamics *dynamics) {
  IniAccess gyro_conf(fname);
  char GSection[30] = "GYRO";

  Quaternion q_b2c;
  gyro_conf.ReadQuaternion(GSection, "q_b2c", q_b2c);
  int prescaler = gyro_conf.ReadInt(GSection, "prescaler");
  if (prescaler <= 1)
    prescaler = 1;

  // SensorBase
  Vector<kGyroDim * kGyroDim> sf_vec;
  gyro_conf.ReadVector(GSection, "ScaleFactor", sf_vec);
  Matrix<kGyroDim, kGyroDim> scale_factor;
  for (size_t i = 0; i < kGyroDim; i++) {
    for (size_t j = 0; j < kGyroDim; j++) {
      scale_factor[i][j] = sf_vec[i * kGyroDim + j];
    }
  }
  double range_to_const = gyro_conf.ReadDouble(GSection, "Range_to_const");
  Vector<kGyroDim> range_to_const_c{range_to_const};
  double range_to_zero = gyro_conf.ReadDouble(GSection, "Range_to_zero");
  Vector<kGyroDim> range_to_zero_c{range_to_zero};

  Vector<kGyroDim> bias_c;
  gyro_conf.ReadVector(GSection, "Bias_c", bias_c);

  double rw_stepwidth = compo_step_time * (double)prescaler;
  Vector<kGyroDim> rw_stddev_c;
  gyro_conf.ReadVector(GSection, "rw_stddev_c", rw_stddev_c);
  Vector<kGyroDim> rw_limit_c;
  gyro_conf.ReadVector(GSection, "rw_limit_c", rw_limit_c);
  Vector<kGyroDim> nr_stddev_c;
  gyro_conf.ReadVector(GSection, "nr_stddev_c", nr_stddev_c);

  SensorBase<kGyroDim> gyro_sb(scale_factor, range_to_const_c, range_to_zero_c,
                               bias_c, nr_stddev_c, rw_stepwidth, rw_stddev_c,
                               rw_limit_c);

  // PowerPort
  double minimum_voltage = gyro_conf.ReadDouble(GSection, "minimum_voltage");
  power_port->SetMinimumVoltage(minimum_voltage);
  double assumed_power_consumption =
      gyro_conf.ReadDouble(GSection, "assumed_power_consumption");
  power_port->SetAssumedPowerConsumption(assumed_power_consumption);

  Gyro gyro(prescaler, clock_gen, power_port, gyro_sb, sensor_id, q_b2c,
            dynamics);
  return gyro;
}
