#include "Spacecraft.h"

#include "../../Interface/LogOutput/LogUtility.h"
#include "../../Interface/LogOutput/Logger.h"

Spacecraft::Spacecraft(SimulationConfig* sim_config, const GlobalEnvironment* glo_env)
{
  Initialize(sim_config, glo_env);
}

Spacecraft::~Spacecraft()
{
  delete dynamics_;
  delete local_env_;
  delete disturbances_;
}

void Spacecraft::Initialize(SimulationConfig* sim_config, const GlobalEnvironment* glo_env)
{
  //Structure
  auto conf = IniAccess(sim_config->ini_base_fname_);
  conf.ReadVector("STRUCTURE", "cg_b", cg_b_);
  mass_ = conf.ReadDouble("STRUCTURE", "mass");
  string ini_dist = conf.ReadString("SIM_SETTING", "dist_file");
  surfaces_ = InitSurfaces(ini_dist);

  clock_gen_.ClearTimerCount();
  local_env_ = new LocalEnvironment(sim_config, glo_env);
  dynamics_ = new Dynamics(sim_config, &(glo_env->GetSimTime()), &(local_env_->GetCelesInfo()));
  disturbances_ = new Disturbances(sim_config->ini_base_fname_,surfaces_);  
}

void Spacecraft::LogSetup(Logger& logger)
{
  dynamics_->LogSetup(logger);
  local_env_->LogSetup(logger);
  disturbances_->LogSetup(logger);
}

void Spacecraft::Update(const SimTime* sim_time)
{
  // Update local environment and disturbance
  local_env_->Update(dynamics_, sim_time);
  disturbances_->Update(*local_env_,*dynamics_);
  // Add generated force and torque by disturbances
  dynamics_->AddAcceleration_i(disturbances_->GetAccelerationI());
  dynamics_->AddTorque_b(disturbances_->GetTorque());
  dynamics_->AddForce_b(disturbances_->GetForce());
  // Propagate dynamics
  dynamics_->Update(sim_time, &(local_env_->GetCelesInfo()));
}

void Spacecraft::Clear(void)
{
  dynamics_->ClearForceTorque();
}