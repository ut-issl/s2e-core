/*
 * @file battery.cpp
 * @brief Component emulation of battery
 */

#include "battery.hpp"

#include <cmath>

BAT::BAT(const int prescaler, ClockGenerator* clock_generator, int number_of_series, int number_of_parallel, double cell_capacity_Ah,
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

BAT::BAT(ClockGenerator* clock_generator, int number_of_series, int number_of_parallel, double cell_capacity_Ah,
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

BAT::BAT(const BAT& obj)
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

BAT::~BAT() {}

std::string BAT::GetLogHeader() const {
  std::string str_tmp = "";
  std::string component_name = "battery_";
  str_tmp += WriteScalar(component_name + "voltage", "V");
  str_tmp += WriteScalar(component_name + "dod", "%");
  return str_tmp;
}

std::string BAT::GetLogValue() const {
  std::string str_tmp = "";
  str_tmp += WriteScalar(battery_voltage_V_);
  str_tmp += WriteScalar(depth_of_discharge_percent_);
  return str_tmp;
}

void BAT::MainRoutine(int time_count) {
  UNUSED(time_count);

  double delta_time_query = compo_step_time_s_ * prescaler_;
  depth_of_discharge_percent_ -= charge_current_A_ * delta_time_query / 3600.0 / (cell_capacity_Ah_ * number_of_parallel_) * 100.0;
  UpdateBatVoltage();
}

void BAT::UpdateBatVoltage() {
  double cell_discharge_capasity = depth_of_discharge_percent_ / 100.0 * cell_capacity_Ah_;
  double temp = 0.0;
  int index = 0;
  for (auto coeff : cell_discharge_curve_coefficients_) {
    temp += coeff * std::pow(cell_discharge_capasity, index);
    ++index;
  }
  battery_voltage_V_ = temp * number_of_series_;
}
