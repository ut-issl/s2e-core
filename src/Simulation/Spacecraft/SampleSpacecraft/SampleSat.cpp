#include "SampleSat.h"
#include "SampleComponents.h"
#include "../../../Interface/InitInput/Initialize.h"
#include "../../../Environment/Global/ClockGenerator.h"
#include "../../../Library/math/NormalRand.hpp"

SampleSat::SampleSat(SimulationConfig* sim_config, const GlobalEnvironment* glo_env, const int sat_id)
  :Spacecraft(sim_config, glo_env, sat_id)
{
  Initialize(sim_config, sat_id);
}

SampleSat::~SampleSat()
{
  delete components_;
}

void SampleSat::Initialize(SimulationConfig* sim_config, const int sat_id)
{
  components_ = new SampleComponents(dynamics_, structure_, sim_config, &clock_gen_, sat_id);
}

void SampleSat::LogSetup(Logger & logger)
{
  Spacecraft::LogSetup(logger);
  components_->CompoLogSetUp(logger);
}

void SampleSat::Update(const SimTime* sim_time)
{
  Spacecraft::Update(sim_time);
  clock_gen_.UpdateComponents(sim_time);
}

void SampleSat::GenerateTorque_b()
{
  dynamics_->AddTorque_b(components_->GenerateTorque_b());
}

void SampleSat::GenerateForce_b()
{
  dynamics_->AddForce_b(components_->GenerateForce_b());
}


