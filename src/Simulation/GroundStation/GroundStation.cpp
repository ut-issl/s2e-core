#include "GroundStation.h"

#include <Interface/InitInput/IniAccess.h>
#include <Interface/LogOutput/LogUtility.h>
#include <Interface/LogOutput/Logger.h>

#include <Environment/Global/PhysicalConstants.hpp>
#include <Library/math/Constant.hpp>
#include <Library/utils/Macros.hpp>
#include <string>

GroundStation::GroundStation(SimulationConfig* config, int gs_id) : gs_id_(gs_id) { Initialize(gs_id_, config); }

GroundStation::~GroundStation() {}

void GroundStation::Initialize(int gs_id, SimulationConfig* config) {
  IniAccess iniAccess = IniAccess(config->ini_base_fname_);
  std::string gs_ini_path = iniAccess.ReadString("SIM_SETTING", "gs_file");
  auto conf = IniAccess(gs_ini_path);

  const char* section_base = "GS";
  const std::string section_tmp = section_base + std::to_string(static_cast<long long>(gs_id));
  const char* Section = section_tmp.data();

  double latitude_deg = conf.ReadDouble(Section, "latitude");
  double longitude_deg = conf.ReadDouble(Section, "longitude");
  double height_m = conf.ReadDouble(Section, "height");
  gs_position_geo_ = GeodeticPosition(latitude_deg * libra::deg_to_rad, longitude_deg * libra::deg_to_rad, height_m);
  gs_position_ecef_ = gs_position_geo_.CalcEcefPosition();

  elevation_angle_ = conf.ReadDouble(Section, "elevation_angle");
}

void GroundStation::LogSetup(Logger& logger) { UNUSED(logger); }

void GroundStation::Update(const CelestialRotation& celes_rotation) {
  Matrix<3, 3> dcm_ecef2eci = transpose(celes_rotation.GetDCMJ2000toXCXF());
  gs_position_i_ = dcm_ecef2eci * gs_position_ecef_;
}
