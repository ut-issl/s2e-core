#include "../Initialize.h"
#include "../../../Component/AOCS/UWBSensor.h"

UWBSensor InitUWBSensor(int sensor_id)
{
  IniAccess conf("data/ini/component/UWBSensor.ini");
  std::string section = "UWB" + std::to_string(sensor_id);
  const char* csection = section.c_str();

  Vector<3> pos;
  conf.ReadVector(csection, "pos", pos);
  Vector<3> dir;
  conf.ReadVector(csection, "dir", dir);
  Vector<3> axis;
  conf.ReadVector(csection, "axis", axis);

  normalize(dir);
  normalize(axis);
  UWBSensor uwb(sensor_id, pos, dir, axis);

  return uwb;
}

