#pragma once
#include "Disturbance.h"
#include "../Environment/Environment.h"
#include "../Simulation/Spacecraft/Spacecraft.h"

// Abstract class for disturbances which generate acceleration (not force)
class AccelerationDisturbance : public Disturbance, public ILoggable
{
public:
  virtual ~AccelerationDisturbance() { }

  virtual inline void UpdateIfEnabled(Envir& env, const Spacecraft& spacecraft)
  {
    if (IsCalcEnabled) { Update(env, spacecraft); }
    else { acceleration_b_ *= 0; acceleration_i_ *= 0; }
  }

  virtual void Update(Envir& env, const Spacecraft& spacecraft) = 0;
};
