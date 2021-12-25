#include "Spacecraft.h"

#include "../../Interface/LogOutput/LogUtility.h"
#include "../../Interface/LogOutput/Logger.h"


Spacecraft::Spacecraft(SimulationConfig config)
  :config_(config)
{
  Initialize();
}

Spacecraft::Spacecraft(SimulationConfig config, string AttitudeName, string OrbitName)
  :config_(config)
{
  Initialize(AttitudeName, OrbitName);
}

Spacecraft::~Spacecraft()
{
  delete dynamics_;
}

void Spacecraft::Initialize(string AttitudeName, string OrbitName)
{
  dynamics_ = new Dynamics(config_, AttitudeName, OrbitName);
}

void Spacecraft::LogSetup(Logger& logger)
{
  dynamics_->LogSetup(logger);
}

void Spacecraft::Update()
{
  dynamics_->Update();
}

