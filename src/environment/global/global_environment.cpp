/**
 * @file global_environment.cpp
 * @brief Class to manage the global environment
 */

#include "global_environment.hpp"

#include "initialize_global_environment.hpp"
#include "initialize_gnss_satellites.hpp"
#include "library/initialize/initialize_file_access.hpp"

GlobalEnvironment::GlobalEnvironment(SimulationConfiguration* simulation_configuration) { Initialize(simulation_configuration); }

GlobalEnvironment::~GlobalEnvironment() {
  delete simulation_time_;
  delete celestial_information_;
  delete hipparcos_catalogue_;
  delete gnss_satellites_;
}

void GlobalEnvironment::Initialize(SimulationConfiguration* simulation_configuration) {
  // Get ini file path
  IniAccess iniAccess = IniAccess(simulation_configuration->initialize_base_file_name_);
  std::string simulation_time_ini_path = simulation_configuration->initialize_base_file_name_;

  // Initialize
  simulation_time_ = InitSimulationTime(simulation_time_ini_path);
  celestial_information_ = InitCelestialInformation(simulation_configuration->initialize_base_file_name_);
  hipparcos_catalogue_ = InitHipparcosCatalogue(simulation_configuration->initialize_base_file_name_);
  gnss_satellites_ = InitGnssSatellites(simulation_configuration->gnss_file_);

  // Calc initial value
  celestial_information_->UpdateAllObjectsInfo(simulation_time_->GetCurrentTime_jd());
  gnss_satellites_->SetUp(simulation_time_);
}

void GlobalEnvironment::Update() {
  simulation_time_->UpdateTime();
  celestial_information_->UpdateAllObjectsInfo(simulation_time_->GetCurrentTime_jd());
  gnss_satellites_->Update(simulation_time_);
}

void GlobalEnvironment::LogSetup(Logger& logger) {
  logger.AddLogList(simulation_time_);
  logger.AddLogList(celestial_information_);
}

void GlobalEnvironment::Reset(void) { simulation_time_->ResetClock(); }
