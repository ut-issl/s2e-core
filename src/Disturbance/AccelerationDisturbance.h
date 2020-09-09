#pragma once
#include "Disturbance.h"
#include "../Environment/Local/LocalEnvironment.h"
#include "../Dynamics/Dynamics.h"

// Abstract class for disturbances which generate acceleration (not force)
class AccelerationDisturbance : public Disturbance, public ILoggable
{
public:
  virtual ~AccelerationDisturbance() { }

  virtual inline void UpdateIfEnabled(const LocalEnvironment& local_env, const Dynamics& dynamics)
  {
    if (IsCalcEnabled) { Update(local_env, dynamics); }
    else { acceleration_b_ *= 0; acceleration_i_ *= 0; }
  }

  virtual void Update(const LocalEnvironment& local_env, const Dynamics& dynamics) = 0;
};
