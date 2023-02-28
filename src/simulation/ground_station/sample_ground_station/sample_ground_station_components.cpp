/**
 * @file sample_ground_station_components.cpp
 * @brief An example of ground station related components list
 */
#include "sample_ground_station_components.hpp"

#include <library/initialize/initialize_file_access.hpp>

SampleGSComponents::SampleGSComponents(const SimulationConfig* configuration) : configuration_(configuration) {
  IniAccess iniAccess = IniAccess(configuration_->ground_station_file_list_[0]);

  std::string ant_ini_path = iniAccess.ReadString("COMPONENT_FILES", "ground_station_antenna_file");
  configuration_->main_logger_->CopyFileToLogDirectory(ant_ini_path);
  antenna_ = new Antenna(InitAntenna(1, ant_ini_path));
  std::string gscalculator_ini_path = iniAccess.ReadString("COMPONENT_FILES", "ground_station_calculator_file");
  configuration_->main_logger_->CopyFileToLogDirectory(gscalculator_ini_path);
  gs_calculator_ = new GroundStationCalculator(InitGsCalculator(gscalculator_ini_path));
}

SampleGSComponents::~SampleGSComponents() {
  delete antenna_;
  delete gs_calculator_;
}

void SampleGSComponents::CompoLogSetUp(Logger& logger) {
  // logger.AddLogList(ant_);
  logger.AddLogList(gs_calculator_);
}
