/**
 * @file SampleSat.cpp
 * @brief An example of user side spacecraft class
 */

#include "SampleSat.h"

#include <Environment/Global/ClockGenerator.h>

#include <Library/math/NormalRand.hpp>

#include "sample_components.hpp"

SampleSat::SampleSat(SimulationConfig* sim_config, const GlobalEnvironment* glo_env, const int sat_id) : Spacecraft(sim_config, glo_env, sat_id) {
  sample_components_ = new SampleComponents(dynamics_, structure_, local_env_, glo_env, sim_config, &clock_gen_, sat_id);
  components_ = sample_components_;
}
