/**
 * @file spacecraft.cpp
 * @brief Definition of Spacecraft class
 */

#include "spacecraft.hpp"

#include <library/logger/log_utility.hpp>
#include <library/logger/logger.hpp>

Spacecraft::Spacecraft(const SimulationConfiguration* simulation_configuration, const GlobalEnvironment* global_environment, const int spacecraft_id)
    : spacecraft_id_(spacecraft_id) {
  Initialize(simulation_configuration, global_environment, spacecraft_id);
}

Spacecraft::Spacecraft(const SimulationConfiguration* simulation_configuration, const GlobalEnvironment* global_environment,
                       RelativeInformation* relative_information, const int spacecraft_id)
    : spacecraft_id_(spacecraft_id) {
  Initialize(simulation_configuration, global_environment, relative_information, spacecraft_id);
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

void Spacecraft::Initialize(const SimulationConfiguration* simulation_configuration, const GlobalEnvironment* global_environment,
                            const int spacecraft_id) {
  clock_generator_.ClearTimerCount();
  structure_ = new Structure(simulation_configuration, spacecraft_id);
  local_environment_ = new LocalEnvironment(simulation_configuration, global_environment, spacecraft_id);
  dynamics_ = new Dynamics(simulation_configuration, &(global_environment->GetSimulationTime()), &(local_environment_->GetCelestialInformation()),
                           spacecraft_id, structure_);
  disturbances_ = new Disturbances(simulation_configuration, spacecraft_id, structure_, global_environment);

  simulation_configuration->main_logger_->CopyFileToLogDirectory(simulation_configuration->spacecraft_file_list_[spacecraft_id]);

  relative_information_ = nullptr;
}

void Spacecraft::Initialize(const SimulationConfiguration* simulation_configuration, const GlobalEnvironment* global_environment,
                            RelativeInformation* relative_information, const int spacecraft_id) {
  clock_generator_.ClearTimerCount();
  structure_ = new Structure(simulation_configuration, spacecraft_id);
  local_environment_ = new LocalEnvironment(simulation_configuration, global_environment, spacecraft_id);
  dynamics_ = new Dynamics(simulation_configuration, &(global_environment->GetSimulationTime()), &(local_environment_->GetCelestialInformation()),
                           spacecraft_id, structure_, relative_information);
  disturbances_ = new Disturbances(simulation_configuration, spacecraft_id, structure_, global_environment);

  simulation_configuration->main_logger_->CopyFileToLogDirectory(simulation_configuration->spacecraft_file_list_[spacecraft_id]);

  relative_information_ = relative_information;
  relative_information_->RegisterDynamicsInfo(spacecraft_id, dynamics_);
}

void Spacecraft::LogSetup(Logger& logger) {
  dynamics_->LogSetup(logger);
  local_environment_->LogSetup(logger);
  disturbances_->LogSetup(logger);
  components_->LogSetup(logger);
}

void Spacecraft::Update(const SimulationTime* simulation_time) {
  dynamics_->ClearForceTorque();

  // Update local environment and disturbance
  local_environment_->Update(dynamics_, simulation_time);
  disturbances_->Update(*local_environment_, *dynamics_, simulation_time);

  // Update components
  clock_generator_.UpdateComponents(simulation_time);

  // Add generated force and torque by disturbances
  dynamics_->AddAcceleration_i_m_s2(disturbances_->GetAcceleration_i_m_s2());
  dynamics_->AddTorque_b_Nm(disturbances_->GetTorque_b_Nm());
  dynamics_->AddForce_b_N(disturbances_->GetForce_b_N());

  // Add generated force and torque by components
  dynamics_->AddTorque_b_Nm(components_->GenerateTorque_b_Nm());
  dynamics_->AddForce_b_N(components_->GenerateForce_b_N());

  // Propagate dynamics
  dynamics_->Update(simulation_time, &(local_environment_->GetCelestialInformation()));
}

void Spacecraft::Clear(void) { dynamics_->ClearForceTorque(); }
