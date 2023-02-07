/**
 * @file ground_station.cpp
 * @brief Base class of ground station
 */

#include "ground_station.hpp"

#include <Interface/InitInput/IniAccess.h>
#include <Interface/LogOutput/LogUtility.h>
#include <Interface/LogOutput/Logger.h>

#include <Environment/Global/PhysicalConstants.hpp>
#include <Library/math/Constant.hpp>
#include <Library/utils/Macros.hpp>
#include <string>

GroundStation::GroundStation(SimulationConfig* config, int gs_id) : gs_id_(gs_id) {
  Initialize(gs_id_, config);
  num_sc_ = config->num_of_simulated_spacecraft_;
  for (int i = 0; i < num_sc_; i++) {
    is_visible_[i] = false;
  }
}

GroundStation::~GroundStation() {}

void GroundStation::Initialize(int gs_id, SimulationConfig* config) {
  std::string gs_ini_path = config->gs_file_;
  auto conf = IniAccess(gs_ini_path);

  const char* section_base = "GROUND_STATION_";
  const std::string section_tmp = section_base + std::to_string(static_cast<long long>(gs_id));
  const char* Section = section_tmp.data();

  double latitude_deg = conf.ReadDouble(Section, "latitude_deg");
  double longitude_deg = conf.ReadDouble(Section, "longitude_deg");
  double height_m = conf.ReadDouble(Section, "height_m");
  gs_position_geo_ = GeodeticPosition(latitude_deg * libra::deg_to_rad, longitude_deg * libra::deg_to_rad, height_m);
  gs_position_ecef_ = gs_position_geo_.CalcEcefPosition();

  elevation_limit_angle_deg_ = conf.ReadDouble(Section, "elevation_limit_angle_deg");

  config->main_logger_->CopyFileToLogDir(gs_ini_path);
}

void GroundStation::LogSetup(Logger& logger) { logger.AddLoggable(this); }

void GroundStation::Update(const CelestialRotation& celes_rotation, const Spacecraft& spacecraft) {
  Matrix<3, 3> dcm_ecef2eci = transpose(celes_rotation.GetDCMJ2000toXCXF());
  gs_position_i_ = dcm_ecef2eci * gs_position_ecef_;

  is_visible_[spacecraft.GetSatID()] = CalcIsVisible(spacecraft.GetDynamics().GetOrbit().GetSatPosition_ecef());
}

bool GroundStation::CalcIsVisible(const Vector<3> sc_pos_ecef_m) {
  Quaternion q_ecef_to_ltc = gs_position_geo_.GetQuaternionXcxfToLtc();

  Vector<3> sc_pos_ltc = q_ecef_to_ltc.frame_conv(sc_pos_ecef_m - gs_position_ecef_);  // Satellite position in LTC frame [m]
  normalize(sc_pos_ltc);
  Vector<3> dir_gs_to_zenith = Vector<3>(0);
  dir_gs_to_zenith[2] = 1;

  // Judge the satellite position angle is over the minimum elevation

  if (dot(sc_pos_ltc, dir_gs_to_zenith) > sin(elevation_limit_angle_deg_ * libra::deg_to_rad)) {
    return true;
  } else {
    return false;
  }
}

std::string GroundStation::GetLogHeader() const {
  std::string str_tmp = "";

  std::string head = "ground_station" + std::to_string(gs_id_) + "_";
  for (int i = 0; i < num_sc_; i++) {
    std::string legend = head + "sc" + std::to_string(i) + "_visible_flag";
    str_tmp += WriteScalar(legend);
  }
  return str_tmp;
}

std::string GroundStation::GetLogValue() const {
  std::string str_tmp = "";

  for (int i = 0; i < num_sc_; i++) {
    str_tmp += WriteScalar(is_visible_.at(i));
  }
  return str_tmp;
}
