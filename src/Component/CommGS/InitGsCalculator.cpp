/*
 * @file InitGscalculator.cpp
 * @brief Initialize function for Ground Station Calculator
 */

#define _CRT_SECURE_NO_WARNINGS
#include "InitGsCalculator.hpp"

#include <string.h>

#include "interface/InitInput/IniAccess.h"

GScalculator InitGScalculator(const std::string fname) {
  IniAccess gs_conf(fname);

  char Section[30] = "GROUND_STATION_CALCULATOR";

  double loss_polarization = gs_conf.ReadDouble(Section, "loss_polarization_dB");
  double loss_atmosphere = gs_conf.ReadDouble(Section, "loss_atmosphere_dB");
  double loss_rainfall = gs_conf.ReadDouble(Section, "loss_rainfall_dB");
  double loss_others = gs_conf.ReadDouble(Section, "loss_others_dB");
  double EbN0 = gs_conf.ReadDouble(Section, "ebn0_dB");
  double hardware_deterioration = gs_conf.ReadDouble(Section, "hardware_deterioration_dB");
  double coding_gain = gs_conf.ReadDouble(Section, "coding_gain_dB");
  double margin_req = gs_conf.ReadDouble(Section, "margin_requirement_dB");
  double downlink_bitrate_bps = gs_conf.ReadDouble(Section, "downlink_bitrate_bps");

  GScalculator gs_calculator(loss_polarization, loss_atmosphere, loss_rainfall, loss_others, EbN0, hardware_deterioration, coding_gain, margin_req,
                             downlink_bitrate_bps);
  return gs_calculator;
}
