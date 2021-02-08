#include "../Initialize.h"
#include <string.h>
#include "../../../Component/AOCS/GNSSReceiver.h"

GNSSReceiver InitGNSSReceiver(ClockGenerator* clock_gen, int id, const string fname, const Dynamics* dynamics, const GnssSatellites* gnss_satellites, const SimTime* simtime)
{
  IniAccess gnssr_conf(fname);
  char GSection[30] = "GNSSReceiver";

  int prescaler = gnssr_conf.ReadInt(GSection, "prescaler");
  if (prescaler <= 1) prescaler = 1;

  AntennaModel antenna_model = static_cast<AntennaModel>(gnssr_conf.ReadInt(GSection, "antenna_model"));
  Vector<3> antenna_pos_b;
  gnssr_conf.ReadVector(GSection, "antenna_pos_b", antenna_pos_b);
  Quaternion q_b2c;
  gnssr_conf.ReadQuaternion(GSection, "q_b2c", q_b2c);
  double half_width = gnssr_conf.ReadDouble(GSection, "half_width");
  string gnss_id = gnssr_conf.ReadString(GSection, "gnss_id");
  int ch_max = gnssr_conf.ReadInt(GSection, "ch_max");
  Vector<3> noise_std;
  gnssr_conf.ReadVector(GSection, "nr_stddev_eci", noise_std);

  GNSSReceiver gnss_r(prescaler, clock_gen, id, gnss_id, ch_max, antenna_model, antenna_pos_b, q_b2c, half_width, noise_std, dynamics, gnss_satellites, simtime);
  return gnss_r;
};


GNSSReceiver InitGNSSReceiver(ClockGenerator* clock_gen, PowerPort* power_port, int id, const string fname, const Dynamics* dynamics, const GnssSatellites* gnss_satellites, const SimTime* simtime)
{
  IniAccess gnssr_conf(fname);
  char GSection[30] = "GNSSReceiver";

  int prescaler = gnssr_conf.ReadInt(GSection, "prescaler");
  if (prescaler <= 1) prescaler = 1;

  AntennaModel antenna_model = static_cast<AntennaModel>(gnssr_conf.ReadInt(GSection, "antenna_model"));
  Vector<3> antenna_pos_b;
  gnssr_conf.ReadVector(GSection, "antenna_pos_b", antenna_pos_b);
  Quaternion q_b2c;
  gnssr_conf.ReadQuaternion(GSection, "q_b2c", q_b2c);
  double half_width = gnssr_conf.ReadDouble(GSection, "half_width");
  string gnss_id = gnssr_conf.ReadString(GSection, "gnss_id");
  int ch_max = gnssr_conf.ReadInt(GSection, "ch_max");
  Vector<3> noise_std;
  gnssr_conf.ReadVector(GSection, "nr_stddev_eci", noise_std);

  GNSSReceiver gnss_r(prescaler, clock_gen, power_port, id, gnss_id, ch_max, antenna_model, antenna_pos_b, q_b2c, half_width, noise_std, dynamics, gnss_satellites, simtime);
  return gnss_r;
};