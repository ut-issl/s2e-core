#include "GroundStation.h"

#include <Interface/LogOutput/LogUtility.h>
#include <Interface/LogOutput/Logger.h>

#include <cmath>
#include <string>

GroundStation::GroundStation(SimulationConfig* config, int gs_id) : gs_id_(gs_id) { Initialize(gs_id_, config); }

GroundStation::~GroundStation() {}

void GroundStation::Initialize(int gs_id, SimulationConfig* config) {
  IniAccess iniAccess = IniAccess(config->ini_base_fname_);
  std::string gs_ini_path = iniAccess.ReadString("SIM_SETTING", "gs_file");
  auto conf = IniAccess(gs_ini_path);

  const std::string st_gs_id = std::to_string(static_cast<long long>(gs_id));
  const char* cs = st_gs_id.data();
  char Section[30] = "GS";
  strcat(Section, cs);

  latitude_ = conf.ReadDouble(Section, "latitude");
  longitude_ = conf.ReadDouble(Section, "longitude");
  height_ = conf.ReadDouble(Section, "height");
  elevation_angle_ = conf.ReadDouble(Section, "elevation_angle");
}

void GroundStation::LogSetup(Logger& logger) {}

void GroundStation::Update(const double& current_jd)  // 慣性系での地上局位置を更新する
{
  double current_side = gstime(current_jd);
  double theta = FMod2p(longitude_ * DEG2RAD + current_side);  //[rad]

  double radiusearthkm;
  double f;
  getwgsconst(whichconst_gs, radiusearthkm, f);

  double e2 = f * (2 - f);
  double c = 1 / sqrt(1 - e2 * sin(latitude_ * DEG2RAD) * sin(latitude_ * DEG2RAD));
  double N = c * radiusearthkm * 1000.0;  //[metre]

  double x = (N + height_) * cos(latitude_ * DEG2RAD) * cos(theta);  //[metre]
  double y = (N + height_) * cos(latitude_ * DEG2RAD) * sin(theta);  //[metre]
  double z = (N * (1 - e2) + height_) * sin(latitude_ * DEG2RAD);    //[metre]

  gs_position_i_(0) = x;
  gs_position_i_(1) = y;
  gs_position_i_(2) = z;
}
