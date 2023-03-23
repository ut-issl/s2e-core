/**
 * @file sample_ground_station_components.cpp
 * @brief An example of ground station related components list
 */
#include "sample_ground_station_components.hpp"

#include <library/initialize/initialize_file_access.hpp>

SampleGsComponents::SampleGsComponents(const SimulationConfiguration* configuration) : configuration_(configuration) {
  IniAccess iniAccess = IniAccess(configuration_->ground_station_file_list_[0]);

  std::string ant_ini_path = iniAccess.ReadString("COMPONENT_FILES", "ground_station_antenna_file");
  configuration_->main_logger_->CopyFileToLogDirectory(ant_ini_path);
  antenna_ = new Antenna(InitAntenna(1, ant_ini_path));
  std::string gs_calculator_ini_path = iniAccess.ReadString("COMPONENT_FILES", "ground_station_calculator_file");
  configuration_->main_logger_->CopyFileToLogDirectory(gs_calculator_ini_path);
  gs_calculator_ = new GroundStationCalculator(InitGsCalculator(gs_calculator_ini_path));
}

SampleGsComponents::~SampleGsComponents() {
  delete antenna_;
  delete gs_calculator_;
}

void SampleGsComponents::CompoLogSetUp(Logger& logger) {
  // logger.AddLogList(ant_);
  logger.AddLogList(gs_calculator_);
}
