/*
 * @file pcu_initial_study.cpp
 * @brief Component emulation of Power Control Unit for initial study of spacecraft project
 */

#include "pcu_initial_study.hpp"

#include <cmath>
#include <components/real/power/csv_scenario_interface.hpp>
#include <environment/global/clock_generator.hpp>

PcuInitialStudy::PcuInitialStudy(const int prescaler, ClockGenerator* clock_generator, const std::vector<SAP*> saps, Battery* battery,
                                 double component_step_time_s)
    : Component(prescaler, clock_generator),
      saps_(saps),
      battery_(battery),
      cc_charge_current_C_(battery->GetCcChargeCurrent_C()),
      cv_charge_voltage_V_(battery->GetCvChargeVoltage_V()),
      compo_step_time_s_(component_step_time_s) {
  bus_voltage_V_ = 0.0;
  power_consumption_W_ = 0.0;
}

PcuInitialStudy::PcuInitialStudy(ClockGenerator* clock_generator, const std::vector<SAP*> saps, Battery* battery)
    : Component(10, clock_generator),
      saps_(saps),
      battery_(battery),
      cc_charge_current_C_(battery->GetCcChargeCurrent_C()),
      cv_charge_voltage_V_(battery->GetCvChargeVoltage_V()),
      compo_step_time_s_(0.1) {
  bus_voltage_V_ = 0.0;
  power_consumption_W_ = 0.0;
}

PcuInitialStudy::~PcuInitialStudy() {}

std::string PcuInitialStudy::GetLogHeader() const {
  std::string str_tmp = "";
  std::string component_name = "pcu_initial_study_";
  str_tmp += WriteScalar(component_name + "power_consumption", "W");
  str_tmp += WriteScalar(component_name + "bus_voltage", "V");
  return str_tmp;
}

std::string PcuInitialStudy::GetLogValue() const {
  std::string str_tmp = "";
  str_tmp += WriteScalar(power_consumption_W_);
  str_tmp += WriteScalar(bus_voltage_V_);
  return str_tmp;
}

void PcuInitialStudy::MainRoutine(int time_count) {
  double time_query = compo_step_time_s_ * time_count;
  power_consumption_W_ = CalcPowerConsumption(time_query);  // Should use SimulationTime? time_count may over flow since it is int type,

  UpdateChargeCurrentAndBusVoltage();
  for (auto sap : saps_) {
    sap->SetVoltage_V(16.0);  // Assume MPPT control
  }
}

double PcuInitialStudy::CalcPowerConsumption(double time_query) const {
  if (CsvScenarioInterface::IsCsvScenarioEnabled()) {
    return CsvScenarioInterface::GetPowerConsumption(time_query);
  } else {
    // Examples
    //  if (time_in_sec % 3600 < 600) {
    //    return 10.0;
    //  } else if (time_in_sec % 3600 < 2000) {
    //    return 5.0;
    //  } else if (time_in_sec % 3600 < 2500) {
    //    return 20.0;
    //  } else {
    //    return 5.0;
    //  }
    return 5.0;
  }
}

void PcuInitialStudy::UpdateChargeCurrentAndBusVoltage() {
  double bat_voltage = battery_->GetVoltage_V();
  const double battery_resistance_Ohm = battery_->GetResistance_Ohm();
  double power_generation = 0.0;
  for (auto sap : saps_) {
    power_generation += sap->GetPowerGeneration();
  }
  double current_temp =
      (-bat_voltage + std::sqrt(bat_voltage * bat_voltage + 4.0 * battery_resistance_Ohm * (power_generation - power_consumption_W_))) /
      (2.0 * battery_resistance_Ohm);
  if (current_temp >= cc_charge_current_C_) {
    if (bat_voltage + cc_charge_current_C_ * battery_resistance_Ohm < cv_charge_voltage_V_) {
      // CC Charge
      battery_->SetChargeCurrent(cc_charge_current_C_);
      bus_voltage_V_ = bat_voltage + battery_resistance_Ohm * cc_charge_current_C_;
    } else {
      // CV Charge
      battery_->SetChargeCurrent((cv_charge_voltage_V_ - bat_voltage) / battery_resistance_Ohm);
      bus_voltage_V_ = bat_voltage + battery_resistance_Ohm * (cv_charge_voltage_V_ - bat_voltage) / battery_resistance_Ohm;
    }
  } else {
    if (bat_voltage + current_temp * battery_resistance_Ohm < cv_charge_voltage_V_) {
      // Natural charge or discharge
      battery_->SetChargeCurrent(current_temp);
      bus_voltage_V_ = bat_voltage + battery_resistance_Ohm * current_temp;
    } else {
      // CV Charge
      battery_->SetChargeCurrent((cv_charge_voltage_V_ - bat_voltage) / battery_resistance_Ohm);
      bus_voltage_V_ = bat_voltage + battery_resistance_Ohm * (cv_charge_voltage_V_ - bat_voltage) / battery_resistance_Ohm;
    }
  }
}
