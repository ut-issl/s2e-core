/**
 * @file sample_case.cpp
 * @brief Example of user defined simulation case
 */

#include "sample_case.hpp"

SampleCase::SampleCase(std::string initialise_base_file) : SimulationCase(initialise_base_file) {}

SampleCase::~SampleCase() {
  delete sample_spacecraft_;
  delete sample_ground_station_;
}

void SampleCase::InitializeTargetObjects() {
  // Instantiate the target of the simulation
  // `spacecraft_id` corresponds to the index of `spacecraft_file` in simulation_base.ini
  const int spacecraft_id = 0;
  sample_spacecraft_ = new SampleSpacecraft(&simulation_configuration_, global_environment_, spacecraft_id);
  const int ground_station_id = 0;
  sample_ground_station_ = new SampleGroundStation(&simulation_configuration_, ground_station_id);

  // Register the log output
  sample_spacecraft_->LogSetup(*(simulation_configuration_.main_logger_));
  sample_ground_station_->LogSetup(*(simulation_configuration_.main_logger_));
}

void SampleCase::UpdateTargetObjects() {
  // Spacecraft Update
  sample_spacecraft_->Update(&(global_environment_->GetSimulationTime()));
  // Ground Station Update
  sample_ground_station_->Update(global_environment_->GetCelestialInformation().GetEarthRotation(), *sample_spacecraft_);
}

std::string SampleCase::GetLogHeader() const {
  std::string str_tmp = "";

  str_tmp += WriteScalar("time", "s");

  return str_tmp;
}

std::string SampleCase::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteScalar(global_environment_->GetSimulationTime().GetElapsedTime_s());

  return str_tmp;
}
