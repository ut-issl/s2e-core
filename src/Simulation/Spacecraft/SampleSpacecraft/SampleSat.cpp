#include "SampleSat.h"

#include <Environment/Global/ClockGenerator.h>

#include <Library/math/NormalRand.hpp>

#include "SampleComponents.h"

SampleSat::SampleSat(SimulationConfig* sim_config, const GlobalEnvironment* glo_env, const int sat_id) : Spacecraft(sim_config, glo_env, sat_id) {
  components_ = new SampleComponents(dynamics_, structure_, local_env_, glo_env, sim_config, &clock_gen_, sat_id);
}
