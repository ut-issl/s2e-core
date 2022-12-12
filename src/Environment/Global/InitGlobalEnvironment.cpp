/**
 *@file InitGlobalEnvironment.cpp
 *@brief Initialize functions for classes in global environment
 */
#include "InitGlobalEnvironment.hpp"

#include <Environment/Global/SimTime.h>
#include <Interface/InitInput/IniAccess.h>
#include <SpiceUsr.h>

#include <cassert>

#define CALC_LABEL "calculation"
#define LOG_LABEL "logging"

SimTime* InitSimTime(std::string file_name) {
  IniAccess ini_file(file_name);

  const char* section = "TIME";
  // Parameters about entire simulation
  double end_sec = ini_file.ReadDouble(section, "EndTimeSec");
  double step_sec = ini_file.ReadDouble(section, "StepTimeSec");
  std::string start_ymdhms = ini_file.ReadString(section, "StartYMDHMS");
  double sim_speed = ini_file.ReadDouble(section, "SimulationSpeed");

  // Time step parameters for dynamics propagation
  double attitude_update_interval_sec = ini_file.ReadDouble(section, "AttitudeUpdateIntervalSec");
  double attitude_rk_step_sec = ini_file.ReadDouble(section, "AttitudeRKStepSec");
  double orbit_update_interval_sec = ini_file.ReadDouble(section, "OrbitUpdateIntervalSec");
  double orbit_rk_step_sec = ini_file.ReadDouble(section, "OrbitRKStepSec");
  double thermal_update_interval_sec = ini_file.ReadDouble(section, "ThermalUpdateIntervalSec");
  double thermal_rk_step_sec = ini_file.ReadDouble(section, "ThermalRKStepSec");

  // Time step parameter for component propagation
  double compo_propagate_step_sec = ini_file.ReadDouble(section, "CompoUpdateIntervalSec");

  // Time step parameter for log output
  double log_output_interval_sec = ini_file.ReadDouble(section, "LogOutPutIntervalSec");

  SimTime* simTime = new SimTime(end_sec, step_sec, attitude_update_interval_sec, attitude_rk_step_sec, orbit_update_interval_sec, orbit_rk_step_sec,
                                 thermal_update_interval_sec, thermal_rk_step_sec, compo_propagate_step_sec, log_output_interval_sec,
                                 start_ymdhms.c_str(), sim_speed);

  return simTime;
}

HipparcosCatalogue* InitHipCatalogue(std::string file_name) {
  IniAccess ini_file(file_name);
  const char* section = "HIPPARCOS_CATALOGUE";

  double max_magnitude = ini_file.ReadDouble(section, "max_magnitude");
  std::string catalogue_path = ini_file.ReadString(section, "catalogue_path");

  HipparcosCatalogue* hip_catalogue;
  hip_catalogue = new HipparcosCatalogue(max_magnitude, catalogue_path);
  hip_catalogue->IsCalcEnabled = ini_file.ReadEnable(section, CALC_LABEL);
  hip_catalogue->IsLogEnabled = ini_file.ReadEnable(section, LOG_LABEL);
  hip_catalogue->ReadContents(catalogue_path, ',');

  return hip_catalogue;
}

CelestialInformation* InitCelesInfo(std::string file_name) {
  IniAccess ini_file(file_name);
  const char* section = "PLANET_SELECTION";
  const char* furnsh_section = "FURNSH_PATH";

  // Read SPICE setting
  std::string inertial_frame = ini_file.ReadString(section, "inertial_frame");
  std::string aber_cor = ini_file.ReadString(section, "aberration_correction");
  std::string center_obj = ini_file.ReadString(section, "center_object");

  // SPICE Furnsh
  std::vector<std::string> keywords = {"TLS", "TPC1", "TPC2", "TPC3", "BSP"};
  for (size_t i = 0; i < keywords.size(); i++) {
    std::string fname = ini_file.ReadString(furnsh_section, keywords[i].c_str());
    furnsh_c(fname.c_str());
  }

  // Initialize celestial body list
  const int num_of_selected_body = ini_file.ReadInt(section, "num_of_selected_body");
  int* selected_body = new int[num_of_selected_body];
  for (int i = 0; i < num_of_selected_body; i++) {
    // Convert body name to SPICE ID
    std::string selected_body_i = "selected_body(" + std::to_string(i) + ")";
    char selected_body_temp[30];
    ini_file.ReadChar(section, selected_body_i.c_str(), 30, selected_body_temp);
    SpiceInt planet_id;
    SpiceBoolean found;
    bodn2c_c(selected_body_temp, (SpiceInt*)&planet_id, (SpiceBoolean*)&found);

    // If the object specified in the ini file is not found, exit the program.
    assert(found == SPICETRUE);

    selected_body[i] = planet_id;
  }

  // Read Rotation setting
  RotationMode rotation_mode;
  std::string rotation_mode_temp = ini_file.ReadString(section, "rotation_mode");
  if (rotation_mode_temp == "Idle") {
    rotation_mode = Idle;
  } else if (rotation_mode_temp == "Simple") {
    rotation_mode = Simple;
  } else if (rotation_mode_temp == "Full") {
    rotation_mode = Full;
  } else  // if rotation_mode is neither Idle, Simple, nor Full, set
          // rotation_mode to Idle
  {
    rotation_mode = Idle;
  }

  CelestialInformation* celestial_info;
  celestial_info = new CelestialInformation(inertial_frame, aber_cor, center_obj, rotation_mode, num_of_selected_body, selected_body);

  // log setting
  celestial_info->IsLogEnabled = ini_file.ReadEnable(section, LOG_LABEL);

  return celestial_info;
}
