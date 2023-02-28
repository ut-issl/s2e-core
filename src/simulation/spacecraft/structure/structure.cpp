/**
 * @file structure.cpp
 * @brief Definition of spacecraft structure
 */

#include "structure.hpp"

#include <library/initialize/initialize_file_access.hpp>
#include <simulation/spacecraft/structure/initialize_structure.hpp>

Structure::Structure(const SimulationConfig* simulation_configuration, const int spacecraft_id) {
  Initialize(simulation_configuration, spacecraft_id);
}

Structure::~Structure() {
  delete kinematics_parameters_;
  delete residual_magnetic_moment_;
}

void Structure::Initialize(const SimulationConfig* simulation_configuration, const int spacecraft_id) {
  // Read file name
  IniAccess conf = IniAccess(simulation_configuration->spacecraft_file_list_[spacecraft_id]);
  std::string ini_fname = conf.ReadString("SETTING_FILES", "structure_file");
  // Save ini file
  simulation_configuration->main_logger_->CopyFileToLogDirectory(ini_fname);
  // Initialize
  kinematics_parameters_ = new KinematicsParameters(InitKinematicsParams(ini_fname));
  surfaces_ = InitSurfaces(ini_fname);
  residual_magnetic_moment_ = new ResidualMagneticMoment(InitRMMParams(ini_fname));
}
