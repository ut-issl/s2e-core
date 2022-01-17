#pragma once

#include "../../Interface/LogOutput/Logger.h"
#include "../../Simulation/SimulationConfig.h"
#include "CelestialInformation.h"
#include "GnssSatellites.h"
#include "HipparcosCatalogue.h"
#include "SimTime.h"

class GlobalEnvironment {
public:
  GlobalEnvironment(SimulationConfig *sim_config);
  ~GlobalEnvironment();
  void Initialize(SimulationConfig *sim_config);
  void Update();
  void LogSetup(Logger &logger);
  void Reset(void);

  inline const SimTime &GetSimTime() const { return *sim_time_; }
  inline const CelestialInformation &GetCelesInfo() const {
    return *celes_info_;
  }
  inline const HipparcosCatalogue &GetHippCatalog() const { return *hipp_; }
  inline const GnssSatellites &GetGnssSatellites() const {
    return *gnss_satellites_;
  }

private:
  SimTime *sim_time_;
  CelestialInformation *celes_info_;
  HipparcosCatalogue *hipp_;
  GnssSatellites *gnss_satellites_;
};
