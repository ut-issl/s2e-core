#pragma once
#include <Environment/Global/GlobalEnvironment.h>
#include <Interface/LogOutput/ILoggable.h>
#include <Simulation/MCSim/MCSimExecutor.h>

#include "../SimulationConfig.h"
class Logger;

class SimulationCase : public ILoggable {
 public:
  SimulationCase(std::string ini_base);
  SimulationCase(std::string ini_base, const MCSimExecutor& mc_sim,
                 std::string log_path);  // For MonteCarlo
  virtual ~SimulationCase();

  virtual void Initialize() = 0;
  virtual void Main() = 0;

  virtual std::string GetLogHeader() const = 0;
  virtual std::string GetLogValue() const = 0;

  // Get
  inline SimulationConfig& GetSimConfig() { return sim_config_; }
  inline const GlobalEnvironment& GetGlobalEnvironment() const { return *glo_env_; }

 protected:
  SimulationConfig sim_config_;
  GlobalEnvironment* glo_env_;
};
