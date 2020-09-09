#pragma once

#include "../../Interface/InitInput/Initialize.h"
#include "../../Dynamics/Dynamics.h"
#include "../../Environment/Local/LocalEnvironment.h"
#include "../../Disturbance/Disturbances.h"

class Spacecraft
{
public:
  Spacecraft(SimulationConfig* sim_config, const GlobalEnvironment* glo_env);
  virtual ~Spacecraft();

  // forbidden copy
  Spacecraft(const Spacecraft &) = delete;
  Spacecraft& operator= (const Spacecraft &) = delete;

  //virtual functions
  virtual void Initialize(SimulationConfig* sim_config, const GlobalEnvironment* glo_env);
  virtual void Update(const SimTime* sim_time);
  virtual void Clear(void);
  virtual void LogSetup(Logger& logger);

  //Get functions
  inline const Dynamics& GetDynamics() const { return *dynamics_; }
  inline const LocalEnvironment& GetLocalEnv() const { return *local_env_; }
  inline const Disturbances& GetDisturbances() const { return *disturbances_; }

protected:
  Dynamics* dynamics_;
  LocalEnvironment* local_env_;
  Disturbances*  disturbances_;
};

