/**
 * @file global_environment.cpp
 * @brief Class to manage the global environment
 */

#include "global_environment.hpp"

#include "setting_file_reader/initialize_file_access.hpp"

namespace s2e::environment {

GlobalEnvironment::GlobalEnvironment(const simulation::SimulationConfiguration* simulation_configuration) { Initialize(simulation_configuration); }

GlobalEnvironment::~GlobalEnvironment() {
  delete simulation_time_;
  delete celestial_information_;
  delete hipparcos_catalogue_;
  delete gnss_satellites_;
}

void GlobalEnvironment::Initialize(const simulation::SimulationConfiguration* simulation_configuration) {
  // Get ini file path
  setting_file_reader::IniAccess iniAccess = setting_file_reader::IniAccess(simulation_configuration->initialize_base_file_name_);
  std::string simulation_time_ini_path = simulation_configuration->initialize_base_file_name_;

  // Initialize
  celestial_information_ = InitCelestialInformation(simulation_configuration->initialize_base_file_name_);
  simulation_time_ = InitSimulationTime(simulation_time_ini_path);
  hipparcos_catalogue_ = InitHipparcosCatalogue(simulation_configuration->initialize_base_file_name_);
  gnss_satellites_ = InitGnssSatellites(simulation_configuration->gnss_file_, celestial_information_->GetEarthRotation(), *simulation_time_);

  // Calc initial value
  celestial_information_->UpdateAllObjectsInformation(*simulation_time_);
}

void GlobalEnvironment::Update() {
  simulation_time_->UpdateTime();
  celestial_information_->UpdateAllObjectsInformation(*simulation_time_);
  gnss_satellites_->Update(*simulation_time_);
}

void GlobalEnvironment::LogSetup(logger::Logger& logger) {
  logger.AddLogList(simulation_time_);
  logger.AddLogList(celestial_information_);
  logger.AddLogList(gnss_satellites_);
}

void GlobalEnvironment::Reset(void) { simulation_time_->ResetClock(); }

} // namespace s2e::environment
