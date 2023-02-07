/**
 * @file SampleCase.cpp
 * @brief Example of user defined simulation case
 */

#include "SampleCase.h"

#include "../spacecraft/SampleSpacecraft/SampleSat.h"

using std::cout;
using std::string;

SampleCase::SampleCase(string ini_base) : SimulationCase(ini_base) {}

SampleCase::~SampleCase() { delete sample_sat_; }

void SampleCase::Initialize() {
  // Instantiate the target of the simulation
  // `sat_id` corresponds to the index of `sat_file` in Simbase.ini
  const int sat_id = 0;
  sample_sat_ = new SampleSat(&sim_config_, glo_env_, sat_id);
  const int gs_id = 0;
  sample_gs_ = new SampleGS(&sim_config_, gs_id);

  // Register the log output
  glo_env_->LogSetup(*(sim_config_.main_logger_));
  sample_sat_->LogSetup(*(sim_config_.main_logger_));
  sample_gs_->LogSetup(*(sim_config_.main_logger_));

  // Write headers to the log
  sim_config_.main_logger_->WriteHeaders();

  // Start the simulation
  cout << "\nSimulationDateTime \n";
  glo_env_->GetSimTime().PrintStartDateTime();
}

void SampleCase::Main() {
  glo_env_->Reset();  // for MonteCarlo Sim
  while (!glo_env_->GetSimTime().GetState().finish) {
    // Logging
    if (glo_env_->GetSimTime().GetState().log_output) {
      sim_config_.main_logger_->WriteValues();
    }

    // Global Environment Update
    glo_env_->Update();
    // Spacecraft Update
    sample_sat_->Update(&(glo_env_->GetSimTime()));
    // Ground Station Update
    sample_gs_->Update(glo_env_->GetCelesInfo().GetEarthRotation(), *sample_sat_);

    // Debug output
    if (glo_env_->GetSimTime().GetState().disp_output) {
      cout << "Progresss: " << glo_env_->GetSimTime().GetProgressionRate() << "%\r";
    }
  }
}

string SampleCase::GetLogHeader() const {
  string str_tmp = "";

  str_tmp += WriteScalar("time", "s");
  // str_tmp += WriteVector("position", "i", "m", 3);
  // str_tmp += WriteVector("velocity", "i", "m/s", 3);
  // str_tmp += WriteVector("quaternion", "i2b", "-", 4);
  // str_tmp += WriteVector("omega", "b", "-", 3);

  return str_tmp;
}

string SampleCase::GetLogValue() const {
  string str_tmp = "";

  // auto pos_i = sample_sat->dynamics_->GetOrbit().GetSatPosition_i();
  // auto vel_i = sample_sat->dynamics_->GetOrbit().GetSatVelocity_i();
  // auto quat_i2b = sample_sat->dynamics_->GetAttitude().GetQuaternion_i2b();
  // auto omega_b = sample_sat->dynamics_->GetAttitude().GetOmega_b();

  // Need to match the contents of log with header setting above
  str_tmp += WriteScalar(glo_env_->GetSimTime().GetElapsedSec());
  // str_tmp += WriteVector(pos_i, 16);
  // str_tmp += WriteVector(vel_i, 10);
  // str_tmp += WriteQuaternion(quat_i2b);
  // str_tmp += WriteVector(omega_b, 10);

  return str_tmp;
}
