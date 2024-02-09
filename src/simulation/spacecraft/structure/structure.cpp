/**
 * @file structure.cpp
 * @brief Definition of spacecraft structure
 */

#include "structure.hpp"

#include <setting_file_reader/initialize_file_access.hpp>
#include <simulation/spacecraft/structure/initialize_structure.hpp>

Structure::Structure(const SimulationConfiguration* simulation_configuration, const int spacecraft_id) {
  Initialize(simulation_configuration, spacecraft_id);
}

Structure::~Structure() {
  delete kinematics_parameters_;
  delete residual_magnetic_moment_;
}

void Structure::Initialize(const SimulationConfiguration* simulation_configuration, const int spacecraft_id) {
  // Read file name
  IniAccess conf = IniAccess(simulation_configuration->spacecraft_file_list_[spacecraft_id]);
  std::string ini_fname = conf.ReadString("SETTING_FILES", "structure_file");
  // Save ini file
  simulation_configuration->main_logger_->CopyFileToLogDirectory(ini_fname);
  // Initialize
  kinematics_parameters_ = new KinematicsParameters(InitKinematicsParameters(ini_fname));
  surfaces_ = InitSurfaces(ini_fname);
  residual_magnetic_moment_ = new ResidualMagneticMoment(InitResidualMagneticMoment(ini_fname));
}
