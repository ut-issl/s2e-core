#pragma once
#include "../Dynamics/SimTime.h"
#include "../Interface/LogOutput/Logger.h"

struct SimulationConfig
{
  string mainIniPath;
  SimTime* simTime;
  Logger* logger;
};

