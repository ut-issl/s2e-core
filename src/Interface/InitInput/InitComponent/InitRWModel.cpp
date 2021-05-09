#include "../Initialize.h"
#include "../../../Component/AOCS/RWModel.h"

RWModel InitRWModel(ClockGenerator* clock_gen, int actuator_id, string file_name, double prop_step, double compo_update_step){
  //Access Parameters
  IniAccess rwmodel_conf(file_name);
  const string st_actuator_num = std::to_string(static_cast<long long>(actuator_id));
  const char *cs = st_actuator_num.data();
  string section_tmp = "RW";
  section_tmp += cs;
  const char *RWsection = section_tmp.data();

  //Read ini file
  int prescaler = rwmodel_conf.ReadInt(RWsection, "prescaler");
  if (prescaler <= 1) prescaler = 1;
  int fast_prescaler = rwmodel_conf.ReadInt(RWsection, "fast_prescaler");
  if (fast_prescaler <= 1) fast_prescaler = 1;
  double inertia = rwmodel_conf.ReadDouble(RWsection, "inertia");
  double max_torque = rwmodel_conf.ReadDouble(RWsection, "max_torque");
  double max_velocity = rwmodel_conf.ReadDouble(RWsection, "max_angular_velocity");

  string direction_determination_mode;
  direction_determination_mode = rwmodel_conf.ReadString(RWsection, "direction_determination_mode");
  Quaternion q_b2c;
  if (direction_determination_mode == "QUATERNION")
  {
    rwmodel_conf.ReadQuaternion(RWsection, "q_b2c", q_b2c);
  }
  else //direction_determination_mode == "DIRECTION" 
  {
    Vector<3> direction_b;
    rwmodel_conf.ReadVector(RWsection, "direction_b", direction_b);
    Vector<3> direction_c(0.0); direction_c[2] = 1.0;
    Quaternion q(direction_b, direction_c);
    q_b2c = q;
  }
  
  Vector<3> pos_b;
  rwmodel_conf.ReadVector(RWsection, "pos_b", pos_b);
  double dead_time = rwmodel_conf.ReadDouble(RWsection, "dead_time");
  Vector<3> ordinary_lag_coef(1.0);
  //rwmodel_conf.ReadVector(RWsection, "firstorder_lag_coef", ordinary_lag_coef);　バグが修正できるまで読み込まない
  Vector<3> coasting_lag_coef(1.0);
  //rwmodel_conf.ReadVector(RWsection, "coasting_lag_coef", coasting_lag_coef);　バグが修正できるまで読み込まない

  bool is_calc_jitter_enabled = rwmodel_conf.ReadEnable(RWsection, "calculation");
  bool is_log_jitter_enabled = rwmodel_conf.ReadEnable(RWsection, "logging");
  
  vector<vector<double>> radial_force_harmonics_coef;
  vector<vector<double>> radial_torque_harmonics_coef;
  string radial_force_harmonics_coef_path = rwmodel_conf.ReadString(RWsection, "radial_force_harmonics_coef_path");
  string radial_torque_harmonics_coef_path = rwmodel_conf.ReadString(RWsection, "radial_torque_harmonics_coef_path");
  int harmonics_degree = rwmodel_conf.ReadInt(RWsection, "harmonics_degree");
  IniAccess conf_radial_force_harmonics(radial_force_harmonics_coef_path);
  IniAccess conf_radial_torque_harmonics(radial_torque_harmonics_coef_path);
  conf_radial_force_harmonics.ReadCsvDouble(radial_force_harmonics_coef, harmonics_degree);
  conf_radial_torque_harmonics.ReadCsvDouble(radial_torque_harmonics_coef, harmonics_degree);

  double structural_resonance_freq = rwmodel_conf.ReadDouble(RWsection, "structural_resonance_freq");
  double damping_factor = rwmodel_conf.ReadDouble(RWsection, "damping_factor");
  double bandwidth = rwmodel_conf.ReadDouble(RWsection, "bandwidth");
  bool considers_structural_resonance = rwmodel_conf.ReadEnable(RWsection, "considers_structural_resonance");

  bool drive_flag = rwmodel_conf.ReadBoolean(RWsection, "motor_drive_init");
  double init_velocity = rwmodel_conf.ReadDouble(RWsection, "angular_velocity_init");

  // Calc periods
  double step_width = prop_step;
  double dt_main_routine = prescaler * compo_update_step;
  double jitter_update_interval = fast_prescaler * compo_update_step;

  RWModel rwmodel(prescaler, fast_prescaler, clock_gen, step_width, dt_main_routine, jitter_update_interval,
	  inertia, max_torque, max_velocity, q_b2c, pos_b, dead_time,
	  ordinary_lag_coef, coasting_lag_coef, 
    is_calc_jitter_enabled, is_log_jitter_enabled, radial_force_harmonics_coef, radial_torque_harmonics_coef, structural_resonance_freq, damping_factor, bandwidth, considers_structural_resonance,
    drive_flag, init_velocity);

  return rwmodel;
}

RWModel InitRWModel(ClockGenerator* clock_gen, PowerPort* power_port, int actuator_id, string file_name, double prop_step, double compo_update_step){
  //Access Parameters
  IniAccess rwmodel_conf(file_name);
  const string st_actuator_num = std::to_string(static_cast<long long>(actuator_id));
  const char *cs = st_actuator_num.data();
  string section_tmp = "RW";
  section_tmp += cs;
  const char *RWsection = section_tmp.data();

  //Read ini file
  int prescaler = rwmodel_conf.ReadInt(RWsection, "prescaler");
  if (prescaler <= 1) prescaler = 1;
  int fast_prescaler = rwmodel_conf.ReadInt(RWsection, "fast_prescaler");
  if (fast_prescaler <= 1) fast_prescaler = 1;
  double inertia = rwmodel_conf.ReadDouble(RWsection, "inertia");
  double max_torque = rwmodel_conf.ReadDouble(RWsection, "max_torque");
  double max_velocity = rwmodel_conf.ReadDouble(RWsection, "max_angular_velocity");

  string direction_determination_mode;
  direction_determination_mode = rwmodel_conf.ReadString(RWsection, "direction_determination_mode");
  Quaternion q_b2c;
  if (direction_determination_mode == "QUATERNION")
  {
    rwmodel_conf.ReadQuaternion(RWsection, "q_b2c", q_b2c);
  }
  else //direction_determination_mode == "DIRECTION" 
  {
    Vector<3> direction_b;
    rwmodel_conf.ReadVector(RWsection, "direction_b", direction_b);
    Vector<3> direction_c(0.0); direction_c[2] = 1.0;
    Quaternion q(direction_b, direction_c);
    q_b2c = q.conjugate();//coordinate transformation
  }

  Vector<3> pos_b;
  rwmodel_conf.ReadVector(RWsection, "pos_b", pos_b);
  double dead_time = rwmodel_conf.ReadDouble(RWsection, "dead_time");
  Vector<3> ordinary_lag_coef(1.0);
  //rwmodel_conf.ReadVector(RWsection, "firstorder_lag_coef", ordinary_lag_coef);　バグが修正できるまで読み込まない
  Vector<3> coasting_lag_coef(1.0);
  //rwmodel_conf.ReadVector(RWsection, "coasting_lag_coef", coasting_lag_coef);　バグが修正できるまで読み込まない
  
  bool is_calc_jitter_enabled = rwmodel_conf.ReadEnable(RWsection, "calculation");
  bool is_log_jitter_enabled = rwmodel_conf.ReadEnable(RWsection, "logging");

  vector<vector<double>> radial_force_harmonics_coef;
  vector<vector<double>> radial_torque_harmonics_coef;
  string radial_force_harmonics_coef_path = rwmodel_conf.ReadString(RWsection, "radial_force_harmonics_coef_path");
  string radial_torque_harmonics_coef_path = rwmodel_conf.ReadString(RWsection, "radial_torque_harmonics_coef_path");
  int harmonics_degree = rwmodel_conf.ReadInt(RWsection, "harmonics_degree");
  IniAccess conf_radial_force_harmonics(radial_force_harmonics_coef_path);
  IniAccess conf_radial_torque_harmonics(radial_torque_harmonics_coef_path);
  conf_radial_force_harmonics.ReadCsvDouble(radial_force_harmonics_coef, harmonics_degree);
  conf_radial_torque_harmonics.ReadCsvDouble(radial_torque_harmonics_coef, harmonics_degree);

  double structural_resonance_freq = rwmodel_conf.ReadDouble(RWsection, "structural_resonance_freq");
  double damping_factor = rwmodel_conf.ReadDouble(RWsection, "damping_factor");
  double bandwidth = rwmodel_conf.ReadDouble(RWsection, "bandwidth");
  bool considers_structural_resonance = rwmodel_conf.ReadEnable(RWsection, "considers_structural_resonance");
  
  bool drive_flag = rwmodel_conf.ReadBoolean(RWsection, "motor_drive_init");
  double init_velocity = rwmodel_conf.ReadDouble(RWsection, "angular_velocity_init");

  // Calc periods
  double step_width = prop_step;
  double dt_main_routine = prescaler * compo_update_step;
  double jitter_update_interval = fast_prescaler * compo_update_step;

  RWModel rwmodel(prescaler, fast_prescaler, clock_gen, power_port, step_width, dt_main_routine, jitter_update_interval,
	  inertia, max_torque, max_velocity, q_b2c, pos_b, dead_time,
	  ordinary_lag_coef, coasting_lag_coef,
    is_calc_jitter_enabled, is_log_jitter_enabled, radial_force_harmonics_coef, radial_torque_harmonics_coef, structural_resonance_freq, damping_factor, bandwidth, considers_structural_resonance,
    drive_flag, init_velocity);

  return rwmodel;
}