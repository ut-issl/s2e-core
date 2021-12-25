#pragma once

#include "../Interface/InitInput/Initialize.h"
#include "Atmosphere.h"
#include "MagEnvironment.h"
#include "SRPEnvironment.h"
class Spacecraft;
class Logger;
class SimTime;

class Envir
{
public:
  Envir(string base_ini_fname);
  ~Envir();
  void Initialize();
  void Update(Spacecraft & spacecraft, SimTime & simTime);
  void LogSetup(Logger& logger);
  Atmosphere* atmosphere;
  MagEnvironment *magnet;
  SRPEnvironment* srp;

private:
  string ini_fname_;
  string base_ini_fname_;
};
