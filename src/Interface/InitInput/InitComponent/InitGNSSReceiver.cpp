#include "../Initialize.h"
#include <string.h>
#include "../../../Component/AOCS/GNSSReceiver.h"

GNSSReceiver InitGNSSReceiver(ClockGenerator* clock_gen, int id, const string fname, const Dynamics* dynamics) {
  IniAccess gnssr_conf(fname);
  char GSection[30] = "GNSSReceiver";

  int prescaler = gnssr_conf.ReadInt(GSection, "prescaler");
  if (prescaler <= 1) prescaler = 1;

  Vector<3> antenna_direction;
  gnssr_conf.ReadVector(GSection, "antenna_direction", antenna_direction);
  Vector<3> noise_std;
  gnssr_conf.ReadVector(GSection, "nr_stddev_eci", noise_std);

  GNSSReceiver gnss_r(prescaler, clock_gen, id, antenna_direction, noise_std,dynamics);
  return gnss_r;
};

GNSSReceiver InitGNSSReceiver(ClockGenerator* clock_gen, PowerPort* power_port, int id, const string fname, const Dynamics* dynamics) {
  IniAccess gnssr_conf(fname);
  char GSection[30] = "GNSSReceiver";

  int prescaler = gnssr_conf.ReadInt(GSection, "prescaler");
  if (prescaler <= 1) prescaler = 1;

  Vector<3> antenna_direction;
  gnssr_conf.ReadVector(GSection, "antenna_direction", antenna_direction);
  Vector<3> noise_std;
  gnssr_conf.ReadVector(GSection, "nr_stddev_eci", noise_std);

  GNSSReceiver gnss_r(prescaler, clock_gen, power_port, id, antenna_direction, noise_std,dynamics);
  return gnss_r;
};