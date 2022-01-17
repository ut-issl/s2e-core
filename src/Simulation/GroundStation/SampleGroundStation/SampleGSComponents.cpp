#include "SampleGSComponents.h"

SampleGSComponents::SampleGSComponents(const SimulationConfig *config)
    : config_(config) {
  IniAccess iniAccess = IniAccess(config_->ini_base_fname_);

  std::string ant_ini_path =
      iniAccess.ReadString("COMPONENTS_FILE", "ant_gs_file");
  config_->main_logger_->CopyFileToLogDir(ant_ini_path);
  ant_ = new ANT(InitANT(2, ant_ini_path));
  std::string gscalculator_ini_path =
      iniAccess.ReadString("COMPONENTS_FILE", "gscalculator_file");
  config_->main_logger_->CopyFileToLogDir(gscalculator_ini_path);
  gscalculator_ = new GScalculator(InitGScalculator(
      gscalculator_ini_path)); // GScalcはGSごとに固有のものなのでidは不要か
}

SampleGSComponents::~SampleGSComponents() {
  delete ant_;
  delete gscalculator_;
}

void SampleGSComponents::CompoLogSetUp(Logger &logger) {
  // logger.AddLoggable(ant_);
  logger.AddLoggable(gscalculator_);
}
