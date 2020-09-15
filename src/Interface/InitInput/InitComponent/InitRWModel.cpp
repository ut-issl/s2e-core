#include "../Initialize.h"
#include "../../../Component/AOCS/RWModel.h"

RWModel InitRWModel(ClockGenerator* clock_gen, int actuator_num, string file_name, double prop_step){

  double step_width = prop_step;
  IniAccess rwmodel_conf(file_name);
  const string st_actuator_num = std::to_string(static_cast<long long>(actuator_num));
  const char *cs = st_actuator_num.data();
  string section_tmp = "RW";
  section_tmp += cs;
  const char *RWsection = section_tmp.data();

  double angular_velocity_init = rwmodel_conf.ReadDouble(RWsection, "angular_velocity_init");

  double inertia = rwmodel_conf.ReadDouble(RWsection, "inertia");

  double angular_velocity_limit_init = rwmodel_conf.ReadDouble(RWsection, "angular_velocity_upperlimit_init");
  double rpm_coasting_end = rwmodel_conf.ReadDouble(RWsection, "angular_velocity_lowerlimit_init");

  bool motor_drive_init = rwmodel_conf.ReadBoolean(RWsection, "motor_drive_init");

  Vector<3> torque_transition;
  rwmodel_conf.ReadVector(RWsection, "torque_transition", torque_transition);


  Vector<3> ordinary_lag_coef;
  rwmodel_conf.ReadVector(RWsection, "firstorder_lag_const", ordinary_lag_coef);

  Vector<3> coasting_lag_coef;
  rwmodel_conf.ReadVector(RWsection, "coasting_lag_coef", coasting_lag_coef);

  double dead_time = rwmodel_conf.ReadDouble(RWsection, "dead_time");
  double max_torque = rwmodel_conf.ReadDouble(RWsection, "max_torque");

  RWModel rwmodel(clock_gen, step_width,
	  angular_velocity_init,
	  inertia,
	  angular_velocity_limit_init,
	  motor_drive_init,
	  torque_transition,
	  dead_time,
	  rpm_coasting_end,
	  ordinary_lag_coef,
	  coasting_lag_coef,
	  max_torque);
  return rwmodel;
}