#include "BAT.h"

#include <cmath>

BAT::BAT(const int prescaler, ClockGenerator* clock_gen, int number_of_series, int number_of_parallel, double cell_capacity,
         const std::vector<double> cell_discharge_curve_coeffs, double initial_dod, double cc_charge_c_rate, double cv_charge_voltage,
         double bat_resistance, double compo_step_time)
    : ComponentBase(prescaler, clock_gen),
      number_of_series_(number_of_series),
      number_of_parallel_(number_of_parallel),
      cell_capacity_(cell_capacity),
      cell_discharge_curve_coeffs_(cell_discharge_curve_coeffs),
      cc_charge_current_(cc_charge_c_rate * cell_capacity * number_of_parallel),
      cv_charge_voltage_(cv_charge_voltage),
      dod_(initial_dod),
      bat_resistance_(bat_resistance),
      compo_step_time_(compo_step_time) {
  charge_current_ = 0.0;
  UpdateBatVoltage();
}

BAT::BAT(ClockGenerator* clock_gen, int number_of_series, int number_of_parallel, double cell_capacity,
         const std::vector<double> cell_discharge_curve_coeffs, double initial_dod, double cc_charge_c_rate, double cv_charge_voltage,
         double bat_resistance)
    : ComponentBase(10, clock_gen),
      number_of_series_(number_of_series),
      number_of_parallel_(number_of_parallel),
      cell_capacity_(cell_capacity),
      cell_discharge_curve_coeffs_(cell_discharge_curve_coeffs),
      cc_charge_current_(cc_charge_c_rate * cell_capacity * number_of_parallel),
      cv_charge_voltage_(cv_charge_voltage),
      dod_(initial_dod),
      bat_resistance_(bat_resistance),
      compo_step_time_(0.1) {
  charge_current_ = 0.0;
  UpdateBatVoltage();
}

BAT::BAT(const BAT& obj)
    : ComponentBase(obj),
      number_of_series_(obj.number_of_series_),
      number_of_parallel_(obj.number_of_parallel_),
      cell_capacity_(obj.cell_capacity_),
      cell_discharge_curve_coeffs_(obj.cell_discharge_curve_coeffs_),
      cc_charge_current_(obj.cc_charge_current_),
      cv_charge_voltage_(obj.cv_charge_voltage_),
      dod_(obj.dod_),
      bat_resistance_(obj.bat_resistance_),
      compo_step_time_(obj.compo_step_time_) {
  charge_current_ = 0.0;
  UpdateBatVoltage();
}

BAT::~BAT() {}

void BAT::SetChargeCurrent(const double current) { charge_current_ = current; }

double BAT::GetBatVoltage() const { return bat_voltage_; }

double BAT::GetBatResistance() const { return bat_resistance_; }

double BAT::GetCCChargeCurrent() const { return cc_charge_current_; }

double BAT::GetCVChargeVoltage() const { return cv_charge_voltage_; }

std::string BAT::GetLogHeader() const {
  std::string str_tmp = "";
  str_tmp += WriteScalar("bat_voltage", "V");
  str_tmp += WriteScalar("DoD", "%");
  return str_tmp;
}

std::string BAT::GetLogValue() const {
  std::string str_tmp = "";
  str_tmp += WriteScalar(bat_voltage_);
  str_tmp += WriteScalar(dod_);
  return str_tmp;
}

void BAT::MainRoutine(int time_count) {
  UNUSED(time_count);

  double delta_time_query = compo_step_time_ * prescaler_;
  dod_ -= charge_current_ * delta_time_query / 3600.0 / (cell_capacity_ * number_of_parallel_) * 100.0;
  UpdateBatVoltage();
}

void BAT::UpdateBatVoltage() {
  double cell_discharge_capasity = dod_ / 100.0 * cell_capacity_;
  double temp = 0.0;
  int index = 0;
  for (auto coeff : cell_discharge_curve_coeffs_) {
    temp += coeff * std::pow(cell_discharge_capasity, index);
    ++index;
  }
  bat_voltage_ = temp * number_of_series_;
}
