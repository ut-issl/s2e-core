#include "GlobalEnvironment.h"
#include "../../Interface/InitInput/Initialize.h"

GlobalEnvironment::GlobalEnvironment(SimulationConfig* sim_config)
{
  Initialize(sim_config);
}

GlobalEnvironment::~GlobalEnvironment()
{
  delete sim_time_;
  delete celes_info_;
  delete hipp_;
}

void GlobalEnvironment::Initialize(SimulationConfig* sim_config)
{
  //Get ini file path
  IniAccess iniAccess = IniAccess(sim_config->ini_base_fname_);
  string sim_time_ini_path = sim_config->ini_base_fname_;
  string celes_info_ini_path = iniAccess.ReadString("SIM_SETTING", "celestial_file");
  string hipp_ini_path = iniAccess.ReadString("SIM_SETTING", "env_file");

  //Initialize
  sim_time_ = InitSimTime(sim_time_ini_path);
  celes_info_ = InitCelesInfo(celes_info_ini_path);
  hipp_ = InitHipCatalogue(hipp_ini_path);

  //Save ini file
  sim_config->main_logger_->CopyFileToLogDir(celes_info_ini_path);
  sim_config->main_logger_->CopyFileToLogDir(hipp_ini_path);

  //Calc initial value
  celes_info_->UpdateAllObjectsInfo(sim_time_->GetCurrentJd());
}

void GlobalEnvironment::Update()
{
  sim_time_->UpdateTime();
  celes_info_->UpdateAllObjectsInfo(sim_time_->GetCurrentJd());
}

void GlobalEnvironment::LogSetup(Logger& logger)
{
  logger.AddLoggable(sim_time_);
  logger.AddLoggable(celes_info_);
}

void GlobalEnvironment::Reset(void)
{
  sim_time_->ResetClock();
}