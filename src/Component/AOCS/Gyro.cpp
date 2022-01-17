#include "Gyro.h"
#include "../CDH/OBC_C2A.h"

Gyro::Gyro(const int prescaler, ClockGenerator *clock_gen,
           SensorBase &sensor_base, const int sensor_id,
           const Quaternion &q_b2c, const Dynamics *dynamics)
    : ComponentBase(prescaler, clock_gen), SensorBase(sensor_base),
      sensor_id_(sensor_id), q_b2c_(q_b2c), dynamics_(dynamics) {}

Gyro::Gyro(const int prescaler, ClockGenerator *clock_gen,
           PowerPort *power_port, SensorBase &sensor_base, const int sensor_id,
           const libra::Quaternion &q_b2c, const Dynamics *dynamics)
    : ComponentBase(prescaler, clock_gen, power_port), SensorBase(sensor_base),
      sensor_id_(sensor_id), q_b2c_(q_b2c), dynamics_(dynamics) {}

Gyro::~Gyro() {}

void Gyro::MainRoutine(int count) {
  omega_c_ =
      q_b2c_.frame_conv(dynamics_->GetAttitude().GetOmega_b()); // Convert frame
  omega_c_ = Measure(omega_c_);                                 // Add noises
}

std::string Gyro::GetLogHeader() const {
  std::string str_tmp = "";
  const std::string st_sensor_id =
      std::to_string(static_cast<long long>(sensor_id_));
  const char *cs = st_sensor_id.data();
  std::string GSection = "gyro_omega";
  str_tmp += WriteVector(GSection + cs, "c", "rad/s", kGyroDim);

  return str_tmp;
}

std::string Gyro::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteVector(omega_c_);

  return str_tmp;
}
