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
  double inertia = rwmodel_conf.ReadDouble(RWsection, "inertia");
  double max_torque = rwmodel_conf.ReadDouble(RWsection, "max_torque");
  double max_velocity = rwmodel_conf.ReadDouble(RWsection, "max_angular_velocity");
  Vector<3> direction_b;
  rwmodel_conf.ReadVector(RWsection, "direction_b", direction_b);
  double dead_time = rwmodel_conf.ReadDouble(RWsection, "dead_time");
  Vector<3> ordinary_lag_coef(1.0);
  //rwmodel_conf.ReadVector(RWsection, "firstorder_lag_coef", ordinary_lag_coef);　バグが修正できるまで読み込まない
  Vector<3> coasting_lag_coef(1.0);
  //rwmodel_conf.ReadVector(RWsection, "coasting_lag_coef", coasting_lag_coef);　バグが修正できるまで読み込まない
  bool drive_flag = rwmodel_conf.ReadBoolean(RWsection, "motor_drive_init");
  double init_velocity = rwmodel_conf.ReadDouble(RWsection, "angular_velocity_init");

  // Calc periods
  double step_width = prop_step;
  double dt_main_routine = prescaler * compo_update_step;

  RWModel rwmodel(prescaler, clock_gen, step_width, dt_main_routine,
	  inertia, max_torque, max_velocity, direction_b, dead_time,
	  ordinary_lag_coef, coasting_lag_coef,drive_flag, init_velocity);

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
  double inertia = rwmodel_conf.ReadDouble(RWsection, "inertia");
  double max_torque = rwmodel_conf.ReadDouble(RWsection, "max_torque");
  double max_velocity = rwmodel_conf.ReadDouble(RWsection, "max_angular_velocity");
  Vector<3> direction_b;
  rwmodel_conf.ReadVector(RWsection, "direction_b", direction_b);
  double dead_time = rwmodel_conf.ReadDouble(RWsection, "dead_time");
  Vector<3> ordinary_lag_coef(1.0);
  //rwmodel_conf.ReadVector(RWsection, "firstorder_lag_coef", ordinary_lag_coef);　バグが修正できるまで読み込まない
  Vector<3> coasting_lag_coef(1.0);
  //rwmodel_conf.ReadVector(RWsection, "coasting_lag_coef", coasting_lag_coef);　バグが修正できるまで読み込まない
  bool drive_flag = rwmodel_conf.ReadBoolean(RWsection, "motor_drive_init");
  double init_velocity = rwmodel_conf.ReadDouble(RWsection, "angular_velocity_init");

  // Calc periods
  double step_width = prop_step;
  double dt_main_routine = prescaler * compo_update_step;

  RWModel rwmodel(prescaler, clock_gen, step_width, dt_main_routine,
	  inertia, max_torque, max_velocity, direction_b, dead_time,
	  ordinary_lag_coef, coasting_lag_coef,drive_flag, init_velocity);

  return rwmodel;
}