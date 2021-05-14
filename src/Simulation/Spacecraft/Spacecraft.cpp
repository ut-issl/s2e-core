#include "Spacecraft.h"

#include "../../Interface/LogOutput/LogUtility.h"
#include "../../Interface/LogOutput/Logger.h"

Spacecraft::Spacecraft(SimulationConfig * sim_config, const GlobalEnvironment * glo_env, const int sat_id) :sat_id_(sat_id)
{
  Initialize(sim_config, glo_env, sat_id);
}

Spacecraft::Spacecraft(SimulationConfig* sim_config, const GlobalEnvironment* glo_env, RelativeInformation* rel_info, InterSatComm* inter_sat_comm, const int sat_id):sat_id_(sat_id)
{
  Initialize(sim_config, glo_env, rel_info, inter_sat_comm, sat_id);
}

Spacecraft::~Spacecraft()
{
  delete structure_;
  delete dynamics_;
  delete local_env_;
  delete disturbances_;
}

void Spacecraft::Initialize(SimulationConfig * sim_config, const GlobalEnvironment * glo_env, const int sat_id)
{
  clock_gen_.ClearTimerCount();
  structure_ = new Structure(sim_config, sat_id);
  local_env_ = new LocalEnvironment(sim_config, glo_env, sat_id);
  dynamics_ = new Dynamics(sim_config, &(glo_env->GetSimTime()), &(local_env_->GetCelesInfo()), sat_id, structure_);
  disturbances_ = new Disturbances(sim_config, sat_id, structure_);

  sim_config->main_logger_->CopyFileToLogDir(sim_config->sat_file_[sat_id]);
}

void Spacecraft::Initialize(SimulationConfig* sim_config, const GlobalEnvironment* glo_env, RelativeInformation* rel_info, InterSatComm* inter_sat_comm, const int sat_id)
{
  clock_gen_.ClearTimerCount();
  structure_ = new Structure(sim_config, sat_id);
  local_env_ = new LocalEnvironment(sim_config, glo_env, sat_id);
  dynamics_ = new Dynamics(sim_config, &(glo_env->GetSimTime()), &(local_env_->GetCelesInfo()), sat_id, structure_);
  disturbances_ = new Disturbances(sim_config, sat_id, structure_); 

  sim_config->main_logger_->CopyFileToLogDir(sim_config->sat_file_[sat_id]);

  rel_info_ = rel_info;
  rel_info_->RegisterDynamicsInfo(sat_id, dynamics_);

  inter_sat_comm_ = inter_sat_comm;
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
  disturbances_->Update(*local_env_,*dynamics_, sim_time);
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