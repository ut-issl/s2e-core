#include "InitSimpleThruster.hpp"

#include <Library/math/Constant.hpp>

#include "Interface/InitInput/IniAccess.h"

SimpleThruster InitSimpleThruster(ClockGenerator* clock_gen, int thruster_id, const std::string fname, const Structure* structure,
                                  const Dynamics* dynamics) {
  IniAccess thruster_conf(fname);
  std::string sectionstr = "THRUSTER" + std::to_string(thruster_id);
  auto* Section = sectionstr.c_str();

  int prescaler = thruster_conf.ReadInt(Section, "prescaler");
  if (prescaler <= 1) prescaler = 1;

  Vector<3> thruster_pos;
  thruster_conf.ReadVector(Section, "thruster_pos", thruster_pos);

  Vector<3> thruster_dir;
  thruster_conf.ReadVector(Section, "thruster_dir", thruster_dir);

  double max_mag = thruster_conf.ReadDouble(Section, "max_mag");

  double mag_err;
  mag_err = thruster_conf.ReadDouble(Section, "mag_err");

  double deg_err;
  deg_err = thruster_conf.ReadDouble(Section, "dir_err") * libra::pi / 180.0;

  SimpleThruster thruster(prescaler, clock_gen, thruster_id, thruster_pos, thruster_dir, max_mag, mag_err, deg_err, structure, dynamics);
  return thruster;
}

SimpleThruster InitSimpleThruster(ClockGenerator* clock_gen, PowerPort* power_port, int thruster_id, const std::string fname,
                                  const Structure* structure, const Dynamics* dynamics) {
  IniAccess thruster_conf(fname);
  std::string sectionstr = "THRUSTER" + std::to_string(thruster_id);
  auto* Section = sectionstr.c_str();

  int prescaler = thruster_conf.ReadInt(Section, "prescaler");
  if (prescaler <= 1) prescaler = 1;

  Vector<3> thruster_pos;
  thruster_conf.ReadVector(Section, "thruster_pos", thruster_pos);

  Vector<3> thruster_dir;
  thruster_conf.ReadVector(Section, "thruster_dir", thruster_dir);

  double max_mag = thruster_conf.ReadDouble(Section, "max_mag");

  double mag_err;
  mag_err = thruster_conf.ReadDouble(Section, "mag_err");

  double deg_err;
  deg_err = thruster_conf.ReadDouble(Section, "dir_err") * libra::pi / 180.0;

  SimpleThruster thruster(prescaler, clock_gen, power_port, thruster_id, thruster_pos, thruster_dir, max_mag, mag_err, deg_err, structure, dynamics);
  return thruster;
}
