#pragma once

#include <Component/CommGS/GScalculator.h>
#include <Dynamics/Dynamics.h>
#include <Environment/Global/GlobalEnvironment.h>

#include <Component/CommGS/Antenna.hpp>

#include "../GroundStation.h"

class SampleGSComponents;

class SampleGS : public GroundStation {
 public:
  SampleGS(SimulationConfig* config, int gs_id);
  ~SampleGS();

  virtual void Initialize(SimulationConfig* config);
  virtual void LogSetup(Logger& logger);
  virtual void Update(const Spacecraft& spacecraft, const GlobalEnvironment& global_env, const Antenna& sc_ant, const SampleGS& sample_gs);

 private:
  SampleGSComponents* components_;
};
