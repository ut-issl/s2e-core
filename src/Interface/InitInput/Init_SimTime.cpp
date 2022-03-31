#include <Environment/Global/SimTime.h>

#include "Initialize.h"

SimTime* InitSimTime(std::string file_name) {
  IniAccess ini_file(file_name);

  char* section = "TIME";
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
