#include <Component/AOCS/EMDS.h>

#include "../Initialize.h"

EMDS InitEMDS(int actuator_id) {
  IniAccess emds_conf("data/ini/component/EMDS.ini");

  std::string section = "EMDS" + std::to_string(actuator_id);

  Vector<3> displacement;
  emds_conf.ReadVector(section.c_str(), "displacement", displacement);

  Vector<3> mm;

  EMDS emds(mm, displacement);
  return emds;
}
