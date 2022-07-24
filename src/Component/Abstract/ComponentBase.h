#pragma once
#include <Environment/Global/ClockGenerator.h>
#include <Interface/SpacecraftInOut/Ports/PowerPort.h>

#include <Library/utils/Macros.hpp>

#include "ITickable.h"

// Base class for components with clock and power on/off features
class ComponentBase : public ITickable {
 public:
  ComponentBase(int prescaler, ClockGenerator* clock_gen, int fast_prescaler = 1);
  ComponentBase(int prescaler, ClockGenerator* clock_gen, PowerPort* power_port, int fast_prescaler = 1);
  ComponentBase(const ComponentBase& obj);
  virtual ~ComponentBase();

  // The methods to input clock. This will be called periodically.
  virtual void Tick(int count);
  virtual void FastTick(int fast_count);

 protected:
  int prescaler_;           //!< Frequency scale factor for normal update
  int fast_prescaler_ = 1;  //!< Frequency scale factor for fast update

  // The method periodically executed when the power switch is on.
  // The period is decided with the prescaler_ and the base clock.
  virtual void MainRoutine(int time_count) = 0;

  // Method used to calculate high-frequency disturbances(e.g. RW jitter)
  // Override only when high-frequency disturbances need to be calculated.
  virtual void FastUpdate(){};

  // The method executed when the power switch is off.
  virtual void PowerOffRoutine(){};

  ClockGenerator* clock_gen_;
  PowerPort* power_port_;
};
