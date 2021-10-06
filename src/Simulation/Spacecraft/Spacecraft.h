#pragma once

#include "../../Interface/InitInput/Initialize.h"
#include "../../Environment/Global/ClockGenerator.h"
#include "../../Dynamics/Dynamics.h"
#include "../../RelativeInformation/RelativeInformation.h"
#include "../../Environment/Local/LocalEnvironment.h"
#include "../../Disturbance/Disturbances.h"
#include "./Structure/Structure.h"

class Spacecraft
{
public:
  Spacecraft(SimulationConfig* sim_config, const GlobalEnvironment* glo_env, const int sat_id); //For single satellite simulation
  Spacecraft(SimulationConfig* sim_config, const GlobalEnvironment* glo_env, RelativeInformation* rel_info, const int sat_id); //for multi satellite simulation
  virtual ~Spacecraft();

  // forbidden copy
  Spacecraft(const Spacecraft &) = delete;
  Spacecraft& operator= (const Spacecraft &) = delete;

  //virtual functions
  virtual void Initialize(SimulationConfig* sim_config, const GlobalEnvironment* glo_env, const int sat_id); //For single satellite simulation
  virtual void Initialize(SimulationConfig* sim_config, const GlobalEnvironment* glo_env, RelativeInformation* rel_info, const int sat_id); //for multi satellite simulation
  virtual void Update(const SimTime* sim_time);
  virtual void Clear(void);
  virtual void LogSetup(Logger& logger);

  //Get functions
  inline const Dynamics& GetDynamics() const { return *dynamics_; }
  inline const LocalEnvironment& GetLocalEnv() const { return *local_env_; }
  inline const Disturbances& GetDisturbances() const { return *disturbances_; }
  inline const int GetSatID() const { return sat_id_; }

protected:
  ClockGenerator clock_gen_;
  Dynamics* dynamics_;
  RelativeInformation* rel_info_;
  LocalEnvironment* local_env_;
  Disturbances*  disturbances_;
  Structure* structure_;
  const int sat_id_;
};

