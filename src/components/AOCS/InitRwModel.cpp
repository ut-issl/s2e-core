/**
 * @file InitRwModel.cpp
 * @brief Initialize functions for Reaction Wheel
 */
#include "InitRwModel.hpp"

#include <vector>

#include "interface/initialize/initialize_file_access.hpp"

// In order to share processing among initialization functions, variables should also be shared. These variables have internal linkages and cannot be
// referenced from the outside.
namespace {
int prescaler;
int fast_prescaler;
double step_width;
double dt_main_routine;
double jitter_update_interval;
double inertia;
double max_torque;
double max_velocity;
libra::Quaternion q_b2c;
libra::Vector<3> pos_b;
double dead_time;
libra::Vector<3> ordinary_lag_coef(1.0);
libra::Vector<3> coasting_lag_coef(1.0);
bool is_calc_jitter_enabled;
bool is_log_jitter_enabled;
std::vector<std::vector<double>> radial_force_harmonics_coef;
std::vector<std::vector<double>> radial_torque_harmonics_coef;
double structural_resonance_freq;
double damping_factor;
double bandwidth;
bool considers_structural_resonance;
bool drive_flag;
double init_velocity;

void InitParams(int actuator_id, std::string file_name, double prop_step, double compo_update_step) {
  // Access Parameters
  IniAccess rwmodel_conf(file_name);
  const std::string st_actuator_num = std::to_string(static_cast<long long>(actuator_id));
  const char* cs = st_actuator_num.data();
  std::string section_tmp = "REACTION_WHEEL_";
  section_tmp += cs;
  const char* RWsection = section_tmp.data();

  // Read ini file
  prescaler = rwmodel_conf.ReadInt(RWsection, "prescaler");
  if (prescaler <= 1) prescaler = 1;
  fast_prescaler = rwmodel_conf.ReadInt(RWsection, "fast_prescaler");
  if (fast_prescaler <= 1) fast_prescaler = 1;
  inertia = rwmodel_conf.ReadDouble(RWsection, "moment_of_inertia_kgm2");
  max_torque = rwmodel_conf.ReadDouble(RWsection, "max_output_torque_Nm");
  max_velocity = rwmodel_conf.ReadDouble(RWsection, "max_angular_velocity_rpm");

  std::string direction_determination_mode;
  direction_determination_mode = rwmodel_conf.ReadString(RWsection, "direction_determination_mode");
  if (direction_determination_mode == "QUATERNION") {
    rwmodel_conf.ReadQuaternion(RWsection, "quaternion_b2c", q_b2c);
  } else  // direction_determination_mode == "DIRECTION"
  {
    Vector<3> direction_b;
    rwmodel_conf.ReadVector(RWsection, "direction_b", direction_b);
    Vector<3> direction_c(0.0);
    direction_c[2] = 1.0;
    Quaternion q(direction_b, direction_c);
    q_b2c = q.conjugate();
  }

  rwmodel_conf.ReadVector(RWsection, "position_b_m", pos_b);
  dead_time = rwmodel_conf.ReadDouble(RWsection, "dead_time_s");
  // rwmodel_conf.ReadVector(RWsection, "first_order_lag_coefficient", ordinary_lag_coef);　// TODO: Fix bug
  // rwmodel_conf.ReadVector(RWsection, "coasting_lag_coefficient", coasting_lag_coef); // TODO: Fix bug

  is_calc_jitter_enabled = rwmodel_conf.ReadEnable(RWsection, "jitter_calculation");
  is_log_jitter_enabled = rwmodel_conf.ReadEnable(RWsection, "jitter_logging");

  std::string radial_force_harmonics_coef_path = rwmodel_conf.ReadString(RWsection, "radial_force_harmonics_coefficient_file");
  std::string radial_torque_harmonics_coef_path = rwmodel_conf.ReadString(RWsection, "radial_torque_harmonics_coefficient_file");
  int harmonics_degree = rwmodel_conf.ReadInt(RWsection, "harmonics_degree");
  IniAccess conf_radial_force_harmonics(radial_force_harmonics_coef_path);
  IniAccess conf_radial_torque_harmonics(radial_torque_harmonics_coef_path);
  conf_radial_force_harmonics.ReadCsvDouble(radial_force_harmonics_coef, harmonics_degree);
  conf_radial_torque_harmonics.ReadCsvDouble(radial_torque_harmonics_coef, harmonics_degree);

  structural_resonance_freq = rwmodel_conf.ReadDouble(RWsection, "structural_resonance_frequency_Hz");
  damping_factor = rwmodel_conf.ReadDouble(RWsection, "damping_factor");
  bandwidth = rwmodel_conf.ReadDouble(RWsection, "bandwidth");
  considers_structural_resonance = rwmodel_conf.ReadEnable(RWsection, "considers_structural_resonance");

  drive_flag = rwmodel_conf.ReadBoolean(RWsection, "initial_motor_drive_flag");
  init_velocity = rwmodel_conf.ReadDouble(RWsection, "initial_angular_velocity_rad_s");

  // Calc periods
  step_width = prop_step;
  dt_main_routine = prescaler * compo_update_step;
  jitter_update_interval = fast_prescaler * compo_update_step;
}
}  // namespace

RWModel InitRWModel(ClockGenerator* clock_gen, int actuator_id, std::string file_name, double prop_step, double compo_update_step) {
  InitParams(actuator_id, file_name, prop_step, compo_update_step);

  RWModel rwmodel(prescaler, fast_prescaler, clock_gen, actuator_id, step_width, dt_main_routine, jitter_update_interval, inertia, max_torque,
                  max_velocity, q_b2c, pos_b, dead_time, ordinary_lag_coef, coasting_lag_coef, is_calc_jitter_enabled, is_log_jitter_enabled,
                  radial_force_harmonics_coef, radial_torque_harmonics_coef, structural_resonance_freq, damping_factor, bandwidth,
                  considers_structural_resonance, drive_flag, init_velocity);

  return rwmodel;
}

RWModel InitRWModel(ClockGenerator* clock_gen, PowerPort* power_port, int actuator_id, std::string file_name, double prop_step,
                    double compo_update_step) {
  InitParams(actuator_id, file_name, prop_step, compo_update_step);

  power_port->InitializeWithInitializeFile(file_name);

  RWModel rwmodel(prescaler, fast_prescaler, clock_gen, power_port, actuator_id, step_width, dt_main_routine, jitter_update_interval, inertia,
                  max_torque, max_velocity, q_b2c, pos_b, dead_time, ordinary_lag_coef, coasting_lag_coef, is_calc_jitter_enabled,
                  is_log_jitter_enabled, radial_force_harmonics_coef, radial_torque_harmonics_coef, structural_resonance_freq, damping_factor,
                  bandwidth, considers_structural_resonance, drive_flag, init_velocity);

  return rwmodel;
}
