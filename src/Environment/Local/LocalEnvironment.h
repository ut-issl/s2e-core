#pragma once

#include <Interface/InitInput/Initialize.h>

#include "../Global/GlobalEnvironment.h"
#include "Atmosphere.h"
#include "LocalCelestialInformation.h"
#include "MagEnvironment.h"
#include "SRPEnvironment.h"
#include "Simulation/SimulationConfig.h"

class Logger;
class SimTime;

class LocalEnvironment {
 public:
  LocalEnvironment(SimulationConfig* sim_config, const GlobalEnvironment* glo_env, const int sat_id);
  ~LocalEnvironment();
  void Initialize(SimulationConfig* sim_config, const GlobalEnvironment* glo_env, const int sat_id);
  void Update(const Dynamics* dynamics, const SimTime* sim_time);
  void LogSetup(Logger& logger);

  // Get functions
  inline const Atmosphere& GetAtmosphere() const { return *atmosphere_; }
  inline const MagEnvironment& GetMag() const { return *mag_; }
  inline const SRPEnvironment& GetSrp() const { return *srp_; }
  inline const LocalCelestialInformation& GetCelesInfo() const { return *celes_info_; }

 private:
  Atmosphere* atmosphere_;
  MagEnvironment* mag_;
  SRPEnvironment* srp_;
  LocalCelestialInformation* celes_info_;
};
