/*
 * @file initialize_battery.cpp
 * @brief Initialize function of Battery
 */
#define _CRT_SECURE_NO_WARNINGS
#include "initialize_battery.hpp"

#include <string>
#include <vector>

#include "library/initialize/initialize_file_access.hpp"

Battery InitBAT(ClockGenerator* clock_generator, int bat_id, const std::string file_name, double component_step_time_s) {
  IniAccess bat_conf(file_name);

  const std::string st_bat_id = std::to_string(bat_id);
  const char* cs = st_bat_id.data();

  char Section[30] = "BATTERY_";
  strcat(Section, cs);

  int prescaler = bat_conf.ReadInt(Section, "prescaler");
  if (prescaler <= 1) prescaler = 1;

  int number_of_series;
  number_of_series = bat_conf.ReadInt(Section, "number_of_series");

  int number_of_parallel;
  number_of_parallel = bat_conf.ReadInt(Section, "number_of_parallel");

  double cell_capacity_Ah;
  cell_capacity_Ah = bat_conf.ReadDouble(Section, "cell_capacity_Ah");

  int approx_order;
  approx_order = bat_conf.ReadInt(Section, "approximation_order");

  std::vector<double> cell_discharge_curve_coefficients;
  for (int i = 0; i <= approx_order; ++i) {
    cell_discharge_curve_coefficients.push_back(
        bat_conf.ReadDouble(Section, ("cell_discharge_curve_coefficients(" + std::to_string(i) + ")").c_str()));
  }

  double initial_dod;
  initial_dod = bat_conf.ReadDouble(Section, "initial_dod");

  double cc_charge_c_rate;
  cc_charge_c_rate = bat_conf.ReadDouble(Section, "constant_charge_current_A_rate_C");

  double cv_charge_voltage_V;
  cv_charge_voltage_V = bat_conf.ReadDouble(Section, "constant_voltage_charge_voltage_V");

  double battery_resistance_Ohm;
  battery_resistance_Ohm = bat_conf.ReadDouble(Section, "battery_resistance_Ohm");

  Battery battery(prescaler, clock_generator, number_of_series, number_of_parallel, cell_capacity_Ah, cell_discharge_curve_coefficients, initial_dod,
                  cc_charge_c_rate, cv_charge_voltage_V, battery_resistance_Ohm, component_step_time_s);

  return battery;
}
