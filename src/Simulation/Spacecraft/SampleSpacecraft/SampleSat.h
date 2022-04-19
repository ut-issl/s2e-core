#pragma once

#include "../Spacecraft.h"
#include "SampleComponents.h"

class SampleComponents;

class SampleSat : public Spacecraft {
 public:
  SampleSat(SimulationConfig* sim_config, const GlobalEnvironment* glo_env, const int sat_id);
};
