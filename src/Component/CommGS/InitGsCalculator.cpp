/*
 * @file InitGscalculator.cpp
 * @brief Initialize function for Ground Station Calculator
 */

#define _CRT_SECURE_NO_WARNINGS
#include "InitGsCalculator.hpp"

#include <string.h>

#include "Interface/InitInput/IniAccess.h"

GScalculator InitGScalculator(const std::string fname) {
  IniAccess gs_conf(fname);

  // const string st_gs_id = std::to_string(static_cast<long long>(gs_id));
  // const char *cs = st_gs_id.data();

  char Section[30] = "GScalculator";
  // strcat(Section, cs);

  double loss_polarization = gs_conf.ReadDouble(Section, "loss_polarization");
  double loss_atmosphere = gs_conf.ReadDouble(Section, "loss_atmosphere");
  double loss_rainfall = gs_conf.ReadDouble(Section, "loss_rainfall");
  double loss_others = gs_conf.ReadDouble(Section, "loss_others");
  double EbN0 = gs_conf.ReadDouble(Section, "EbN0");
  double hardware_deterioration = gs_conf.ReadDouble(Section, "hardware_deterioration");
  double coding_gain = gs_conf.ReadDouble(Section, "coding_gain");
  double margin_req = gs_conf.ReadDouble(Section, "margin_req");
  double downlink_bitrate_bps = gs_conf.ReadDouble(Section, "downlink_bitrate_bps");

  GScalculator gs_calculator(loss_polarization, loss_atmosphere, loss_rainfall, loss_others, EbN0, hardware_deterioration, coding_gain, margin_req,
                             downlink_bitrate_bps);
  return gs_calculator;
}
