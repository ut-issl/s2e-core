#pragma once
#include "../../Interface/LogOutput/ILoggable.h"
#include "../SimulationConfig.h"
#include "../../Environment/Global/GlobalEnvironment.h"
#include "../../Simulation/MCSim/MCSimExecutor.h"
class Logger;

class SimulationCase : public ILoggable
{
public:
  SimulationCase(string ini_base);
  SimulationCase(string ini_base, const MCSimExecutor& mc_sim, string log_path);  //For MonteCarlo
  virtual ~SimulationCase();

  virtual void Initialize() = 0;
  virtual void Main() = 0;

  virtual string GetLogHeader() const = 0;
  virtual string GetLogValue() const = 0;

  //Get 
  inline SimulationConfig& GetSimConfig(){return sim_config_;}
  inline const GlobalEnvironment& GetGlobalEnvironment() const {return *glo_env_;}

protected:
  SimulationConfig sim_config_;
  GlobalEnvironment* glo_env_;
};


