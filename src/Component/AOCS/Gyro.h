#ifndef Gyro_H_
#define Gyro_H_

#include "../../Dynamics/Dynamics.h"
#include "../../Interface/LogOutput/ILoggable.h"
#include "../../Library/math/Quaternion.hpp"
#include "../Abstract/ComponentBase.h"
#include "../Abstract/SensorBase.h"

const size_t kGyroDim = 3;

class Gyro : public ComponentBase,
             public SensorBase<kGyroDim>,
             public ILoggable {
public:
  Gyro(const int prescaler, ClockGenerator *clock_gen, SensorBase &sensor_base,
       const int sensor_id, const libra::Quaternion &q_b2c,
       const Dynamics *dynamics);
  Gyro(const int prescaler, ClockGenerator *clock_gen, PowerPort *power_port,
       SensorBase &sensor_base, const int sensor_id,
       const libra::Quaternion &q_b2c, const Dynamics *dynamics);
  ~Gyro();
  // ComponentBase
  void MainRoutine(int count) override;
  // ILoggable
  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;
  // Getter
  inline const libra::Vector<kGyroDim> &GetOmegaC(void) const {
    return omega_c_;
  }

protected:
  libra::Vector<kGyroDim> omega_c_{0.0};
  int sensor_id_ = 0;
  libra::Quaternion q_b2c_{
      0.0, 0.0, 0.0, 1.0}; //! Quaternion from body frame to component frame

  const Dynamics *dynamics_;
};

#endif
