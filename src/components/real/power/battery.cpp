/*
 * @file battery.cpp
 * @brief Component emulation of battery
 */

#include "battery.hpp"

#include <cmath>
#include <library/initialize/initialize_file_access.hpp>

Battery::Battery(const int prescaler, ClockGenerator* clock_generator, int number_of_series, int number_of_parallel, double cell_capacity_Ah,
                 const std::vector<double> cell_discharge_curve_coefficients, double initial_dod, double cc_charge_c_rate, double cv_charge_voltage_V,
                 double battery_resistance_Ohm, double component_step_time_s)
    : Component(prescaler, clock_generator),
      number_of_series_(number_of_series),
      number_of_parallel_(number_of_parallel),
      cell_capacity_Ah_(cell_capacity_Ah),
      cell_discharge_curve_coefficients_(cell_discharge_curve_coefficients),
      cc_charge_current_C_(cc_charge_c_rate * cell_capacity_Ah * number_of_parallel),
      cv_charge_voltage_V_(cv_charge_voltage_V),
      depth_of_discharge_percent_(initial_dod),
      battery_resistance_Ohm_(battery_resistance_Ohm),
      compo_step_time_s_(component_step_time_s) {}

Battery::Battery(ClockGenerator* clock_generator, int number_of_series, int number_of_parallel, double cell_capacity_Ah,
                 const std::vector<double> cell_discharge_curve_coefficients, double initial_dod, double cc_charge_c_rate, double cv_charge_voltage_V,
                 double battery_resistance_Ohm)
    : Component(10, clock_generator),
      number_of_series_(number_of_series),
      number_of_parallel_(number_of_parallel),
      cell_capacity_Ah_(cell_capacity_Ah),
      cell_discharge_curve_coefficients_(cell_discharge_curve_coefficients),
      cc_charge_current_C_(cc_charge_c_rate * cell_capacity_Ah * number_of_parallel),
      cv_charge_voltage_V_(cv_charge_voltage_V),
      depth_of_discharge_percent_(initial_dod),
      battery_resistance_Ohm_(battery_resistance_Ohm),
      compo_step_time_s_(0.1) {}

Battery::Battery(const Battery& obj)
    : Component(obj),
      number_of_series_(obj.number_of_series_),
      number_of_parallel_(obj.number_of_parallel_),
      cell_capacity_Ah_(obj.cell_capacity_Ah_),
      cell_discharge_curve_coefficients_(obj.cell_discharge_curve_coefficients_),
      cc_charge_current_C_(obj.cc_charge_current_C_),
      cv_charge_voltage_V_(obj.cv_charge_voltage_V_),
      depth_of_discharge_percent_(obj.depth_of_discharge_percent_),
      battery_resistance_Ohm_(obj.battery_resistance_Ohm_),
      compo_step_time_s_(obj.compo_step_time_s_) {
  charge_current_A_ = 0.0;
  UpdateBatVoltage();
}

Battery::~Battery() {}

std::string Battery::GetLogHeader() const {
  std::string str_tmp = "";
  std::string component_name = "battery_";
  str_tmp += WriteScalar(component_name + "voltage", "V");
  str_tmp += WriteScalar(component_name + "dod", "%");
  return str_tmp;
}

std::string Battery::GetLogValue() const {
  std::string str_tmp = "";
  str_tmp += WriteScalar(battery_voltage_V_);
  str_tmp += WriteScalar(depth_of_discharge_percent_);
  return str_tmp;
}

void Battery::MainRoutine(const int time_count) {
  UNUSED(time_count);

  double delta_time_query = compo_step_time_s_ * prescaler_;
  depth_of_discharge_percent_ -= charge_current_A_ * delta_time_query / 3600.0 / (cell_capacity_Ah_ * number_of_parallel_) * 100.0;
  UpdateBatVoltage();
}

void Battery::UpdateBatVoltage() {
  double cell_discharge_capacity = depth_of_discharge_percent_ / 100.0 * cell_capacity_Ah_;
  double temp = 0.0;
  int index = 0;
  for (auto coeff : cell_discharge_curve_coefficients_) {
    temp += coeff * std::pow(cell_discharge_capacity, index);
    ++index;
  }
  battery_voltage_V_ = temp * number_of_series_;
}

Battery InitBAT(ClockGenerator* clock_generator, int bat_id, const std::string file_name, double component_step_time_s) {
  IniAccess bat_conf(file_name);

  const std::string section_name = "BATTERY_" + std::to_string(static_cast<long long>(bat_id));

  int prescaler = bat_conf.ReadInt(section_name.c_str(), "prescaler");
  if (prescaler <= 1) prescaler = 1;

  int number_of_series;
  number_of_series = bat_conf.ReadInt(section_name.c_str(), "number_of_series");

  int number_of_parallel;
  number_of_parallel = bat_conf.ReadInt(section_name.c_str(), "number_of_parallel");

  double cell_capacity_Ah;
  cell_capacity_Ah = bat_conf.ReadDouble(section_name.c_str(), "cell_capacity_Ah");

  int approx_order;
  approx_order = bat_conf.ReadInt(section_name.c_str(), "approximation_order");

  std::vector<double> cell_discharge_curve_coefficients;
  for (int i = 0; i <= approx_order; ++i) {
    cell_discharge_curve_coefficients.push_back(
        bat_conf.ReadDouble(section_name.c_str(), ("cell_discharge_curve_coefficients(" + std::to_string(i) + ")").c_str()));
  }

  double initial_dod;
  initial_dod = bat_conf.ReadDouble(section_name.c_str(), "initial_dod");

  double cc_charge_c_rate;
  cc_charge_c_rate = bat_conf.ReadDouble(section_name.c_str(), "constant_charge_current_A_rate_C");

  double cv_charge_voltage_V;
  cv_charge_voltage_V = bat_conf.ReadDouble(section_name.c_str(), "constant_voltage_charge_voltage_V");

  double battery_resistance_Ohm;
  battery_resistance_Ohm = bat_conf.ReadDouble(section_name.c_str(), "battery_resistance_Ohm");

  Battery battery(prescaler, clock_generator, number_of_series, number_of_parallel, cell_capacity_Ah, cell_discharge_curve_coefficients, initial_dod,
                  cc_charge_c_rate, cv_charge_voltage_V, battery_resistance_Ohm, component_step_time_s);

  return battery;
}
