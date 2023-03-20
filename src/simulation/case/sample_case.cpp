/**
 * @file sample_case.cpp
 * @brief Example of user defined simulation case
 */

#include "sample_case.hpp"

#include "../spacecraft/sample_spacecraft/sample_spacecraft.hpp"

using std::cout;
using std::string;

SampleCase::SampleCase(string initialise_base_file) : SimulationCase(initialise_base_file) {}

SampleCase::~SampleCase() {
  delete sample_spacecraft_;
  delete sample_ground_station_;
}

void SampleCase::Initialize() {
  // Instantiate the target of the simulation
  // `spacecraft_id` corresponds to the index of `spacecraft_file` in simulation_base.ini
  const int spacecraft_id = 0;
  sample_spacecraft_ = new SampleSpacecraft(&simulation_configuration_, global_environment_, spacecraft_id);
  const int ground_station_id = 0;
  sample_ground_station_ = new SampleGroundStation(&simulation_configuration_, ground_station_id);

  // Register the log output
  global_environment_->LogSetup(*(simulation_configuration_.main_logger_));
  sample_spacecraft_->LogSetup(*(simulation_configuration_.main_logger_));
  sample_ground_station_->LogSetup(*(simulation_configuration_.main_logger_));

  // Write headers to the log
  simulation_configuration_.main_logger_->WriteHeaders();

  // Start the simulation
  cout << "\nSimulationDateTime \n";
  global_environment_->GetSimulationTime().PrintStartDateTime();
}

void SampleCase::Main() {
  global_environment_->Reset();  // for MonteCarlo Simulation
  while (!global_environment_->GetSimulationTime().GetState().finish) {
    // Logging
    if (global_environment_->GetSimulationTime().GetState().log_output) {
      simulation_configuration_.main_logger_->WriteValues();
    }

    // Global Environment Update
    global_environment_->Update();
    // Spacecraft Update
    sample_spacecraft_->Update(&(global_environment_->GetSimulationTime()));
    // Ground Station Update
    sample_ground_station_->Update(global_environment_->GetCelestialInformation().GetEarthRotation(), *sample_spacecraft_);

    // Debug output
    if (global_environment_->GetSimulationTime().GetState().disp_output) {
      cout << "Progress: " << global_environment_->GetSimulationTime().GetProgressionRate() << "%\r";
    }
  }
}

string SampleCase::GetLogHeader() const {
  string str_tmp = "";

  str_tmp += WriteScalar("time", "s");

  return str_tmp;
}

string SampleCase::GetLogValue() const {
  string str_tmp = "";

  str_tmp += WriteScalar(global_environment_->GetSimulationTime().GetElapsedTime_s());

  return str_tmp;
}
