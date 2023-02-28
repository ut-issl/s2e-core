/**
 * @file structure.cpp
 * @brief Definition of spacecraft structure
 */

#include "structure.hpp"

#include <library/initialize/initialize_file_access.hpp>
#include <simulation/spacecraft/structure/initialize_structure.hpp>

Structure::Structure(SimulationConfig* simulation_configuration, const int spacecraft_id) { Initialize(simulation_configuration, spacecraft_id); }

Structure::~Structure() {
  delete kinnematics_params_;
  delete rmm_params_;
}

void Structure::Initialize(SimulationConfig* simulation_configuration, const int spacecraft_id) {
  // Read file name
  IniAccess conf = IniAccess(simulation_configuration->spacecraft_file_list_[spacecraft_id]);
  std::string ini_fname = conf.ReadString("SETTING_FILES", "structure_file");
  // Save ini file
  simulation_configuration->main_logger_->CopyFileToLogDirectory(ini_fname);
  // Initialize
  kinnematics_params_ = new KinematicsParameters(InitKinematicsParams(ini_fname));
  surfaces_ = InitSurfaces(ini_fname);
  rmm_params_ = new ResidualMagneticMoment(InitRMMParams(ini_fname));
}
