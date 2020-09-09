#include "SimulationCase.h"
#include "../../Interface/InitInput/Initialize.h"

SimulationCase::SimulationCase(string ini_base)
{
  sim_config_.ini_base_fname_ = ini_base;
  sim_config_.main_logger_ = InitLog(sim_config_.ini_base_fname_);
  glo_env_ = new GlobalEnvironment(&sim_config_);
}
SimulationCase::SimulationCase(string ini_base, const MCSimExecutor& mc_sim, const string log_path)
{
  sim_config_.ini_base_fname_ = ini_base;
  //Log for Monte Carlo Simulation
  string log_file_name = "default" + to_string(mc_sim.GetNumOfExecutionsDone()) + ".csv";
  //ToDo: Consider that `enable_inilog = false` is fine or not?
  sim_config_.main_logger_ = new Logger(log_file_name, log_path, ini_base, false, mc_sim.LogHistory());
  //Global Environment
  glo_env_ = new GlobalEnvironment(&sim_config_);
}
SimulationCase::~SimulationCase()
{
  delete glo_env_;
}
    