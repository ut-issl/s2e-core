/**
 * @file Spacecraft.cpp
 * @brief Definition of Spacecraft class
 */

#include "Spacecraft.h"

#include <Interface/LogOutput/LogUtility.h>
#include <Interface/LogOutput/Logger.h>

Spacecraft::Spacecraft(SimulationConfig* sim_config, const GlobalEnvironment* glo_env, const int sat_id) : sat_id_(sat_id) {
  Initialize(sim_config, glo_env, sat_id);
}

Spacecraft::Spacecraft(SimulationConfig* sim_config, const GlobalEnvironment* glo_env, RelativeInformation* rel_info, const int sat_id)
    : sat_id_(sat_id) {
  Initialize(sim_config, glo_env, rel_info, sat_id);
}

Spacecraft::~Spacecraft() {
  if (rel_info_ != nullptr) {
    rel_info_->RemoveDynamicsInfo(sat_id_);
  }
  delete structure_;
  delete dynamics_;
  delete local_env_;
  delete disturbances_;
  delete components_;
}

void Spacecraft::Initialize(SimulationConfig* sim_config, const GlobalEnvironment* glo_env, const int sat_id) {
  clock_gen_.ClearTimerCount();
  structure_ = new Structure(sim_config, sat_id);
  local_env_ = new LocalEnvironment(sim_config, glo_env, sat_id);
  dynamics_ = new Dynamics(sim_config, &(glo_env->GetSimTime()), &(local_env_->GetCelesInfo()), sat_id, structure_);
  disturbances_ = new Disturbances(sim_config, sat_id, structure_, glo_env);

  sim_config->main_logger_->CopyFileToLogDir(sim_config->sat_file_[sat_id]);

  rel_info_ = nullptr;
}

void Spacecraft::Initialize(SimulationConfig* sim_config, const GlobalEnvironment* glo_env, RelativeInformation* rel_info, const int sat_id) {
  clock_gen_.ClearTimerCount();
  structure_ = new Structure(sim_config, sat_id);
  local_env_ = new LocalEnvironment(sim_config, glo_env, sat_id);
  dynamics_ = new Dynamics(sim_config, &(glo_env->GetSimTime()), &(local_env_->GetCelesInfo()), sat_id, structure_, rel_info);
  disturbances_ = new Disturbances(sim_config, sat_id, structure_, glo_env);

  sim_config->main_logger_->CopyFileToLogDir(sim_config->sat_file_[sat_id]);

  rel_info_ = rel_info;
  rel_info_->RegisterDynamicsInfo(sat_id, dynamics_);
}

void Spacecraft::LogSetup(Logger& logger) {
  dynamics_->LogSetup(logger);
  local_env_->LogSetup(logger);
  disturbances_->LogSetup(logger);
  components_->LogSetup(logger);
}

void Spacecraft::Update(const SimTime* sim_time) {
  dynamics_->ClearForceTorque();

  // Update local environment and disturbance
  local_env_->Update(dynamics_, sim_time);
  disturbances_->Update(*local_env_, *dynamics_, sim_time);

    // Add generated force and torque by disturbances
  dynamics_->AddAcceleration_i(disturbances_->GetAccelerationI());
  dynamics_->AddTorque_b(disturbances_->GetTorque());
  dynamics_->AddForce_b(disturbances_->GetForce());

  // Add generated force and torque by components
  dynamics_->AddTorque_b(components_->GenerateTorque_Nm_b());
  dynamics_->AddForce_b(components_->GenerateForce_N_b());

  // Propagate dynamics
  dynamics_->Update(sim_time, &(local_env_->GetCelesInfo()));

  // Update components
  clock_gen_.UpdateComponents(sim_time);
}

void Spacecraft::Clear(void) { dynamics_->ClearForceTorque(); }
