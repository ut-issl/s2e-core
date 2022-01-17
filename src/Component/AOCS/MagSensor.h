#ifndef MagSensor_H_
#define MagSensor_H_

#include "../../Environment/Local/LocalEnvironment.h"
#include "../../Interface/LogOutput/ILoggable.h"
#include "../../Library/math/Quaternion.hpp"
#include "../Abstract/ComponentBase.h"
#include "../Abstract/SensorBase.h"

const size_t kMagDim = 3;

class MagSensor : public ComponentBase,
                  public SensorBase<kMagDim>,
                  public ILoggable {
public:
  MagSensor(const int prescaler, ClockGenerator *clock_gen,
            SensorBase &sensor_base, const int sensor_id,
            const libra::Quaternion &q_b2c,
            const MagEnvironment *magnet // nT
  );
  MagSensor(const int prescaler, ClockGenerator *clock_gen,
            PowerPort *power_port, SensorBase &sensor_base, const int sensor_id,
            const libra::Quaternion &q_b2c,
            const MagEnvironment *magnet // nT
  );
  ~MagSensor();
  // ComponentBase
  void MainRoutine(int count) override;
  // ILoggable
  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;
  // Getter
  inline const libra::Vector<kMagDim> &GetMagC(void) const { return mag_c_; }

protected:
  libra::Vector<kMagDim> mag_c_{0.0}; // nT
  int sensor_id_ = 0;
  libra::Quaternion q_b2c_{
      0.0, 0.0, 0.0, 1.0}; //! Quaternion from body frame to component frame

  const MagEnvironment *magnet_;
};

#endif
