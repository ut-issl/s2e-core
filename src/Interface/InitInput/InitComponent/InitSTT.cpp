#include "../Initialize.h"
#include <Component/AOCS/STT.h>
#define _USE_MATH_DEFINES
#include <math.h>

using namespace std;

STT InitSTT(ClockGenerator* clock_gen, int sensor_id, const string fname, double compo_step_time, const Dynamics *dynamics, const LocalEnvironment* local_env){
  IniAccess STT_conf(fname);
  string section_tmp = "STT";
  const char *STTSection = section_tmp.data();

  int prescaler = STT_conf.ReadInt(STTSection, "prescaler");
  if (prescaler <= 1) prescaler = 1;
  double step_time = compo_step_time * prescaler;
  Quaternion q_b2c;
  STT_conf.ReadQuaternion(STTSection, "q_b2c",q_b2c);
  double sigma_ortho = STT_conf.ReadDouble(STTSection, "sigma_ortho");
  double sigma_sight = STT_conf.ReadDouble(STTSection, "sigma_sight");
  double output_delay_sec = STT_conf.ReadDouble(STTSection, "output_delay");
  int output_delay = max(int(output_delay_sec / step_time),1);
  double output_interval_sec = STT_conf.ReadDouble(STTSection, "output_interval");
  int output_interval = max(int(output_interval_sec / step_time),1);
  double sun_forbidden_angle_deg = STT_conf.ReadDouble(STTSection, "sun_forbidden_angle");
  double sun_forbidden_angle_rad = sun_forbidden_angle_deg*M_PI / 180.0;
  double earth_forbidden_angle_deg = STT_conf.ReadDouble(STTSection, "earth_forbidden_angle");
  double earth_forbidden_angle_rad = earth_forbidden_angle_deg*M_PI / 180.0;
  double moon_forbidden_angle_deg = STT_conf.ReadDouble(STTSection, "moon_forbidden_angle");
  double moon_forbidden_angle_rad = moon_forbidden_angle_deg*M_PI / 180.0;
  double capture_rate_deg_s = STT_conf.ReadDouble(STTSection, "capture_rate");
  double capture_rate_rad_s = capture_rate_deg_s*M_PI / 180.0;

  STT stt(prescaler, clock_gen, sensor_id, q_b2c, sigma_ortho, sigma_sight, step_time, output_delay, output_interval,
          sun_forbidden_angle_rad, earth_forbidden_angle_rad, moon_forbidden_angle_rad, capture_rate_rad_s, dynamics, local_env);
  return stt;
}

STT InitSTT(ClockGenerator* clock_gen, PowerPort* power_port, int sensor_id, const string fname, double compo_step_time, const Dynamics *dynamics, const LocalEnvironment* local_env){
  IniAccess STT_conf(fname);
  string section_tmp = "STT";
  const char *STTSection = section_tmp.data();

  int prescaler = STT_conf.ReadInt(STTSection, "prescaler");
  if (prescaler <= 1) prescaler = 1;
  double step_time = compo_step_time * prescaler;
  
  Quaternion q_b2c;
  STT_conf.ReadQuaternion(STTSection, "q_b2c",q_b2c);
  double sigma_ortho = STT_conf.ReadDouble(STTSection, "sigma_ortho");
  double sigma_sight = STT_conf.ReadDouble(STTSection, "sigma_sight");
  double output_delay_sec = STT_conf.ReadDouble(STTSection, "output_delay");
  int output_delay = max(int(output_delay_sec / step_time),1);
  double output_interval_sec = STT_conf.ReadDouble(STTSection, "output_interval");
  int output_interval = max(int(output_interval_sec / step_time),1);
  double sun_forbidden_angle_deg = STT_conf.ReadDouble(STTSection, "sun_forbidden_angle");
  double sun_forbidden_angle_rad = sun_forbidden_angle_deg*M_PI / 180.0;
  double earth_forbidden_angle_deg = STT_conf.ReadDouble(STTSection, "earth_forbidden_angle");
  double earth_forbidden_angle_rad = earth_forbidden_angle_deg*M_PI / 180.0;
  double moon_forbidden_angle_deg = STT_conf.ReadDouble(STTSection, "moon_forbidden_angle");
  double moon_forbidden_angle_rad = moon_forbidden_angle_deg*M_PI / 180.0;
  double capture_rate_deg_s = STT_conf.ReadDouble(STTSection, "capture_rate");
  double capture_rate_rad_s = capture_rate_deg_s*M_PI / 180.0;

  STT stt(prescaler, clock_gen, power_port, sensor_id, q_b2c, sigma_ortho, sigma_sight, step_time, output_delay, output_interval,
          sun_forbidden_angle_rad, earth_forbidden_angle_rad, moon_forbidden_angle_rad, capture_rate_rad_s, dynamics, local_env);
  return stt;
}
