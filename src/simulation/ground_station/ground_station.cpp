/**
 * @file ground_station.cpp
 * @brief Base class of ground station
 */

#include "ground_station.hpp"

#include <environment/global/physical_constants.hpp>
#include <logger/log_utility.hpp>
#include <logger/logger.hpp>
#include <math_physics/math/constants.hpp>
#include <setting_file_reader/initialize_file_access.hpp>
#include <string>
#include <utilities/macros.hpp>

namespace s2e::ground_station {

GroundStation::GroundStation(const simulation::SimulationConfiguration* configuration, const unsigned int ground_station_id)
    : ground_station_id_(ground_station_id) {
  Initialize(configuration, ground_station_id_);
  number_of_spacecraft_ = configuration->number_of_simulated_spacecraft_;
  for (unsigned int i = 0; i < number_of_spacecraft_; i++) {
    is_visible_[i] = false;
  }
}

GroundStation::~GroundStation() {}

void GroundStation::Initialize(const simulation::SimulationConfiguration* configuration, const unsigned int ground_station_id) {
  std::string gs_ini_path = configuration->ground_station_file_list_[0];
  auto conf = setting_file_reader::IniAccess(gs_ini_path);

  const char* section_base = "GROUND_STATION_";
  const std::string section_tmp = section_base + std::to_string(static_cast<long long>(ground_station_id));
  const char* Section = section_tmp.data();

  double latitude_deg = conf.ReadDouble(Section, "latitude_deg");
  double longitude_deg = conf.ReadDouble(Section, "longitude_deg");
  double height_m = conf.ReadDouble(Section, "height_m");
  geodetic_position_ = geodesy::GeodeticPosition(latitude_deg * math::deg_to_rad, longitude_deg * math::deg_to_rad, height_m);
  position_ecef_m_ = geodetic_position_.CalcEcefPosition();

  elevation_limit_angle_deg_ = conf.ReadDouble(Section, "elevation_limit_angle_deg");

  configuration->main_logger_->CopyFileToLogDirectory(gs_ini_path);
}

void GroundStation::LogSetup(logger::Logger& logger) { logger.AddLogList(this); }

void GroundStation::Update(const environment::EarthRotation& celestial_rotation, const simulation::Spacecraft& spacecraft) {
  math::Matrix<3, 3> dcm_ecef2eci = celestial_rotation.GetDcmJ2000ToEcef().Transpose();
  position_i_m_ = dcm_ecef2eci * position_ecef_m_;

  is_visible_[spacecraft.GetSpacecraftId()] = CalcIsVisible(spacecraft.GetDynamics().GetOrbit().GetPosition_ecef_m());
}

bool GroundStation::CalcIsVisible(const math::Vector<3> spacecraft_position_ecef_m) {
  math::Quaternion q_ecef_to_ltc = geodetic_position_.GetQuaternionXcxfToLtc();

  math::Vector<3> sc_pos_ltc = q_ecef_to_ltc.FrameConversion(spacecraft_position_ecef_m - position_ecef_m_);  // Satellite position in LTC frame [m]
  sc_pos_ltc = sc_pos_ltc.CalcNormalizedVector();
  math::Vector<3> dir_gs_to_zenith = math::Vector<3>(0);
  dir_gs_to_zenith[2] = 1;

  // Judge the satellite position angle is over the minimum elevation

  if (dot(sc_pos_ltc, dir_gs_to_zenith) > sin(elevation_limit_angle_deg_ * math::deg_to_rad)) {
    return true;
  } else {
    return false;
  }
}

std::string GroundStation::GetLogHeader() const {
  std::string str_tmp = "";

  std::string head = "ground_station" + std::to_string(ground_station_id_) + "_";
  for (unsigned int i = 0; i < number_of_spacecraft_; i++) {
    std::string legend = head + "sc" + std::to_string(i) + "_visible_flag";
    str_tmp += logger::WriteScalar(legend);
  }
  str_tmp += logger::WriteVector("ground_station_position", "eci", "m", 3);
  return str_tmp;
}

std::string GroundStation::GetLogValue() const {
  std::string str_tmp = "";

  for (unsigned int i = 0; i < number_of_spacecraft_; i++) {
    str_tmp += logger::WriteScalar(is_visible_.at(i));
  }
  str_tmp += logger::WriteVector(position_i_m_);
  return str_tmp;
}

}  // namespace s2e::ground_station
