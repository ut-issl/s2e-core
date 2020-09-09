#include "Initialize.h"
#include "../../Environment/Global/SimTime.h"
#include "../../Dynamics/Orbit/SimpleCircularOrbit.h"
#include "../../Dynamics/Orbit/EarthCenteredOrbit.h"

class EarthCenteredOrbit;
class SimpleCircularOrbit;

Orbit* InitOrbit(string ini_path, double stepSec, double current_jd, string section)
{
  auto conf = IniAccess(ini_path);
  const char* section_ = section.c_str();
  Orbit* orbit;

  int propagate_mode = conf.ReadInt(section_, "propagate_mode");

  //地球周回の軌道情報をinitialize
  if (propagate_mode == 1)
  {
    char tle1[80], tle2[80];
    conf.ReadChar(section_, "tle1", 80, tle1);
    conf.ReadChar(section_, "tle2", 80, tle2);

    int wgs = conf.ReadInt(section_, "wgs");
    orbit = new EarthCenteredOrbit(tle1, tle2, wgs, current_jd);
  }

  //深宇宙の軌道情報をinitialize
  else
  {
    double mu = conf.ReadDouble(section_, "GM_e");
	int wgs = conf.ReadInt(section_, "wgs");
    orbit = new SimpleCircularOrbit(mu, stepSec, wgs);
    Vector<3> init_pos;
    conf.ReadVector<3>(section_, "init_position", init_pos);
    Vector<3> init_veloc;
    conf.ReadVector<3>(section_, "init_velocity", init_veloc);

    orbit->Initialize(init_pos, init_veloc, current_jd);
  }
  orbit->IsCalcEnabled = conf.ReadEnable(section_, CALC_LABEL);
  orbit->IsLogEnabled = conf.ReadEnable(section_, LOG_LABEL);
  return orbit;
}
