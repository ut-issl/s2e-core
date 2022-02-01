#include "GlobalEnvironment.h"
#include <Interface/InitInput/Initialize.h>

GlobalEnvironment::GlobalEnvironment(SimulationConfig* sim_config)
{
  Initialize(sim_config);
}

GlobalEnvironment::~GlobalEnvironment()
{
  delete sim_time_;
  delete celes_info_;
  delete hipp_;
  delete gnss_satellites_;
}

void GlobalEnvironment::Initialize(SimulationConfig* sim_config)
{
  //Get ini file path
  IniAccess iniAccess = IniAccess(sim_config->ini_base_fname_);
  std::string sim_time_ini_path = sim_config->ini_base_fname_;

  //Initialize
  sim_time_ = InitSimTime(sim_time_ini_path);
  celes_info_ = InitCelesInfo(sim_config->ini_base_fname_);
  hipp_ = InitHipCatalogue(sim_config->ini_base_fname_);
  gnss_satellites_ = InitGnssSatellites(sim_config->gnss_file_);

  //Calc initial value
  celes_info_->UpdateAllObjectsInfo(sim_time_->GetCurrentJd());
  gnss_satellites_->SetUp(sim_time_);
}

void GlobalEnvironment::Update()
{
  sim_time_->UpdateTime();
  celes_info_->UpdateAllObjectsInfo(sim_time_->GetCurrentJd());
  gnss_satellites_->Update(sim_time_);
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
