/**
 * @file spacecraft.cpp
 * @brief Definition of Spacecraft class
 */

#include "spacecraft.hpp"

#include <library/logger/log_utility.hpp>
#include <library/logger/logger.hpp>

Spacecraft::Spacecraft(SimulationConfig* sim_config, const GlobalEnvironment* glo_env, const int sat_id) : spacecraft_id_(sat_id) {
  Initialize(sim_config, glo_env, sat_id);
}

Spacecraft::Spacecraft(SimulationConfig* sim_config, const GlobalEnvironment* glo_env, RelativeInformation* rel_info, const int sat_id)
    : spacecraft_id_(sat_id) {
  Initialize(sim_config, glo_env, rel_info, sat_id);
}

Spacecraft::~Spacecraft() {
  if (relative_information_ != nullptr) {
    relative_information_->RemoveDynamicsInfo(spacecraft_id_);
  }
  delete structure_;
  delete dynamics_;
  delete local_environment_;
  delete disturbances_;
  delete components_;
}

void Spacecraft::Initialize(SimulationConfig* sim_config, const GlobalEnvironment* glo_env, const int sat_id) {
  clock_generator_.ClearTimerCount();
  structure_ = new Structure(sim_config, sat_id);
  local_environment_ = new LocalEnvironment(sim_config, glo_env, sat_id);
  dynamics_ = new Dynamics(sim_config, &(glo_env->GetSimulationTime()), &(local_environment_->GetCelestialInformation()), sat_id, structure_);
  disturbances_ = new Disturbances(sim_config, sat_id, structure_, glo_env);

  sim_config->main_logger_->CopyFileToLogDirectory(sim_config->spacecraft_file_list_[sat_id]);

  relative_information_ = nullptr;
}

void Spacecraft::Initialize(SimulationConfig* sim_config, const GlobalEnvironment* glo_env, RelativeInformation* rel_info, const int sat_id) {
  clock_generator_.ClearTimerCount();
  structure_ = new Structure(sim_config, sat_id);
  local_environment_ = new LocalEnvironment(sim_config, glo_env, sat_id);
  dynamics_ =
      new Dynamics(sim_config, &(glo_env->GetSimulationTime()), &(local_environment_->GetCelestialInformation()), sat_id, structure_, rel_info);
  disturbances_ = new Disturbances(sim_config, sat_id, structure_, glo_env);

  sim_config->main_logger_->CopyFileToLogDirectory(sim_config->spacecraft_file_list_[sat_id]);

  relative_information_ = rel_info;
  relative_information_->RegisterDynamicsInfo(sat_id, dynamics_);
}

void Spacecraft::LogSetup(Logger& logger) {
  dynamics_->LogSetup(logger);
  local_environment_->LogSetup(logger);
  disturbances_->LogSetup(logger);
  components_->LogSetup(logger);
}

void Spacecraft::Update(const SimulationTime* sim_time) {
  dynamics_->ClearForceTorque();

  // Update local environment and disturbance
  local_environment_->Update(dynamics_, sim_time);
  disturbances_->Update(*local_environment_, *dynamics_, sim_time);

  // Update components
  clock_generator_.UpdateComponents(sim_time);

  // Add generated force and torque by disturbances
  dynamics_->AddAcceleration_i_m_s2(disturbances_->GetAcceleration_i_m_s2());
  dynamics_->AddTorque_b_Nm(disturbances_->GetTorque_b_Nm());
  dynamics_->AddForce_b_N(disturbances_->GetForce_b_N());

  // Add generated force and torque by components
  dynamics_->AddTorque_b_Nm(components_->GenerateTorque_Nm_b());
  dynamics_->AddForce_b_N(components_->GenerateForce_N_b());

  // Propagate dynamics
  dynamics_->Update(sim_time, &(local_environment_->GetCelestialInformation()));
}

void Spacecraft::Clear(void) { dynamics_->ClearForceTorque(); }
