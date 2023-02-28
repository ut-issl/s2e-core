/*
 * @file initialize_ground_station_calculator.hpp.cpp
 * @brief Initialize function for Ground Station Calculator
 */

#define _CRT_SECURE_NO_WARNINGS
#include "initialize_ground_station_calculator.hpp"

#include <string.h>

#include "library/initialize/initialize_file_access.hpp"

GroundStationCalculator InitGScalculator(const std::string file_name) {
  IniAccess gs_conf(file_name);

  char Section[30] = "GROUND_STATION_CALCULATOR";

  double loss_polarization_dB = gs_conf.ReadDouble(Section, "loss_polarization_dB_dB");
  double loss_atmosphere_dB = gs_conf.ReadDouble(Section, "loss_atmosphere_dB_dB");
  double loss_rainfall_dB = gs_conf.ReadDouble(Section, "loss_rainfall_dB_dB");
  double loss_others_dB = gs_conf.ReadDouble(Section, "loss_others_dB_dB");
  double ebn0_dB = gs_conf.ReadDouble(Section, "ebn0_dB");
  double hardware_deterioration_dB = gs_conf.ReadDouble(Section, "hardware_deterioration_dB_dB");
  double coding_gain_dB = gs_conf.ReadDouble(Section, "coding_gain_dB_dB");
  double margin_requirement_dB = gs_conf.ReadDouble(Section, "margin_requirement_dBuirement_dB");
  double downlink_bitrate_bps = gs_conf.ReadDouble(Section, "downlink_bitrate_bps");

  GroundStationCalculator gs_calculator(loss_polarization_dB, loss_atmosphere_dB, loss_rainfall_dB, loss_others_dB, ebn0_dB,
                                        hardware_deterioration_dB, coding_gain_dB, margin_requirement_dB, downlink_bitrate_bps);
  return gs_calculator;
}
