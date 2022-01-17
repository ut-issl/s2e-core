﻿#define _CRT_SECURE_NO_WARNINGS
#include "../../../Component/Power/BAT.h"
#include "../Initialize.h"
#include <string>
#include <vector>

BAT InitBAT(ClockGenerator *clock_gen, int bat_id, const std::string fname) {

  IniAccess bat_conf(fname);

  const std::string st_bat_id = std::to_string(static_cast<long long>(bat_id));
  const char *cs = st_bat_id.data();

  char Section[30] = "BAT";
  strcat(Section, cs);

  int number_of_series;
  number_of_series = bat_conf.ReadInt(Section, "number_of_series");

  int number_of_parallel;
  number_of_parallel = bat_conf.ReadInt(Section, "number_of_parallel");

  double cell_capacity;
  cell_capacity = bat_conf.ReadDouble(Section, "cell_capacity");

  int approx_order;
  approx_order = bat_conf.ReadInt(Section, "approx_order");

  std::vector<double> cell_discharge_curve_coeffs;
  for (int i = 0; i <= approx_order; ++i) {
    cell_discharge_curve_coeffs.push_back(bat_conf.ReadDouble(
        Section,
        ("cell_discharge_curve_coeffs(" + std::to_string(i) + ")").c_str()));
  }

  double initial_dod;
  initial_dod = bat_conf.ReadDouble(Section, "initial_dod");

  double cc_charge_c_rate;
  cc_charge_c_rate = bat_conf.ReadDouble(Section, "cc_charge_c_rate");

  double cv_charge_voltage;
  cv_charge_voltage = bat_conf.ReadDouble(Section, "cv_charge_voltage");

  double bat_resistance;
  bat_resistance = bat_conf.ReadDouble(Section, "bat_resistance");

  BAT bat(clock_gen, number_of_series, number_of_parallel, cell_capacity,
          cell_discharge_curve_coeffs, initial_dod, cc_charge_c_rate,
          cv_charge_voltage, bat_resistance);

  return bat;
}
