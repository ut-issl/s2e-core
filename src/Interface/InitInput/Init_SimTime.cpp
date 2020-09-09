#include "Initialize.h"
#include "../../Environment/Global/SimTime.h"

SimTime* InitSimTime(string file_name)
{
  IniAccess ini_file(file_name);

  char* section = "TIME";

  double end_sec = ini_file.ReadDouble(section, "EndTimeSec");
  double step_sec = ini_file.ReadDouble(section, "StepTimeSec");
  double orbit_propagate_step_sec = ini_file.ReadDouble(section, "OrbitPropagateStepTimeSec");
  int log_period = ini_file.ReadInt(section, "LogPeriod");
  string start_ymdhms =  ini_file.ReadString(section, "StartYMDHMS");
  double sim_speed = ini_file.ReadDouble(section, "SimulationSpeed");

  SimTime* simTime = new SimTime(end_sec, step_sec, orbit_propagate_step_sec, log_period, start_ymdhms.c_str(), sim_speed);

  return simTime;
}
