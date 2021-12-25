#include "../Initialize.h"
#include <string.h>
#include "../../../Component/AOCS/GNSSReceiver.h"

GNSSReceiver InitGNSSReceiver(int id, const string fname, const Dynamics* dynamics) {
  IniAccess gnssr_conf(fname);
  char GSection[30] = "GNSSReceiver";

  Vector<3> antenna_direction;
  gnssr_conf.ReadVector(GSection, "antenna_direction", antenna_direction);
  Vector<3> noise_std;
  gnssr_conf.ReadVector(GSection, "nr_stddev_eci", noise_std);


  GNSSReceiver gnss_r(id, antenna_direction, noise_std,dynamics);
  return gnss_r;
};