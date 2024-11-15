/**
 * @file sample_ground_station_components.cpp
 * @brief An example of ground station related components list
 */
#include "sample_ground_station_components.hpp"

#include <setting_file_reader/initialize_file_access.hpp>

namespace s2e::sample {

SampleGsComponents::SampleGsComponents(const simulation::SimulationConfiguration* configuration) : configuration_(configuration) {
  setting_file_reader::IniAccess iniAccess = setting_file_reader::IniAccess(configuration_->ground_station_file_list_[0]);

  std::string ant_ini_path = iniAccess.ReadString("COMPONENT_FILES", "ground_station_antenna_file");
  configuration_->main_logger_->CopyFileToLogDirectory(ant_ini_path);
  antenna_ = new components::Antenna(components::InitAntenna(1, ant_ini_path));
  std::string gs_calculator_ini_path = iniAccess.ReadString("COMPONENT_FILES", "ground_station_calculator_file");
  configuration_->main_logger_->CopyFileToLogDirectory(gs_calculator_ini_path);
  gs_calculator_ = new components::GroundStationCalculator(components::InitGsCalculator(gs_calculator_ini_path));
}

SampleGsComponents::~SampleGsComponents() {
  delete antenna_;
  delete gs_calculator_;
}

void SampleGsComponents::CompoLogSetUp(logger::Logger& logger) {
  // logger.AddLogList(ant_);
  logger.AddLogList(gs_calculator_);
}

}  // namespace s2e::sample
