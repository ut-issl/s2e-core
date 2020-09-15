#include "SampleSat.h"
#include "SampleComponents.h"
#include "../../../Interface/InitInput/Initialize.h"
#include "../../../Environment/Global/ClockGenerator.h"
#include "../../../Library/math/NormalRand.hpp"

SampleSat::SampleSat(SimulationConfig* sim_config, const GlobalEnvironment* glo_env)
  :Spacecraft(sim_config, glo_env)
{
  Initialize(sim_config);
}

SampleSat::~SampleSat()
{
  delete components_;
}

void SampleSat::Initialize(SimulationConfig* sim_config)
{
  components_ = new SampleComponents(dynamics_, sim_config, &clock_gen_);
}

void SampleSat::LogSetup(Logger & logger)
{
  Spacecraft::LogSetup(logger);
  components_->CompoLogSetUp(logger);
}

void SampleSat::Update(const SimTime* sim_time)
{
  Spacecraft::Update(sim_time);
  // Component update
  for (int i = 0; i < sim_time->GetStepSec() * 1000; i++)
  {
    clock_gen_.TickToComponents();
  }
}

void SampleSat::GenerateTorque_b()
{
  dynamics_->AddTorque_b(components_->GenerateTorque_b());
}

void SampleSat::GenerateForce_b()
{
  dynamics_->AddForce_b(components_->GenerateForce_b());
}


