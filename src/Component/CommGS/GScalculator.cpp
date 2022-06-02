/*
 * @file GScalculator.cpp
 * @brief Emuration of analysis and calculation for Ground Stations
 * @author 山本 智貴
 */

#include "GScalculator.h"

#include <Environment/Global/PhysicalConstants.hpp>
#include <Library/math/Constant.hpp>

GScalculator::GScalculator(const double loss_polarization, const double loss_atmosphere, const double loss_rainfall, const double loss_others,
                           const double EbN0, const double hardware_deterioration, const double coding_gain, const double margin_req)
    : loss_polarization_(loss_polarization),
      loss_atmosphere_(loss_atmosphere),
      loss_rainfall_(loss_rainfall),
      loss_others_(loss_others),
      EbN0_(EbN0),
      hardware_deterioration_(hardware_deterioration),
      coding_gain_(coding_gain),
      margin_req_(margin_req) {
  is_visible_ = false;
  max_bitrate_ = 0.0f;
}

GScalculator::~GScalculator() {}

void GScalculator::Update(const Dynamics& dynamics, const Antenna& sc_ant, const GroundStation& groundstation, const Antenna& gs_ant) {
  is_visible_ = CalcIsVisible(dynamics, groundstation);
  if (is_visible_) {
    max_bitrate_ = CalcMaxBitrate(dynamics, sc_ant, groundstation, gs_ant);
  } else {
    max_bitrate_ = 0.0f;
  }
}

bool GScalculator::CalcIsVisible(const Dynamics& dynamics, const GroundStation& groundstation) {
  Vector<3> sc_pos_ecef = dynamics.GetOrbit().GetSatPosition_ecef();
  Vector<3> gs_pos_ecef = groundstation.GetGSPosition_ecef();

  double lat = groundstation.GetGSPosition_geo().GetLat_rad();
  double lon = groundstation.GetGSPosition_geo().GetLon_rad();

  // ECEF -> LTC frame transform at the ground station
  Matrix<3, 3> trans_mat;
  trans_mat[0][0] = -sin(lon);
  trans_mat[0][1] = cos(lon);
  trans_mat[0][2] = 0;
  trans_mat[1][0] = -sin(lat) * cos(lon);
  trans_mat[1][1] = -sin(lat) * sin(lon);
  trans_mat[1][2] = cos(lat);
  trans_mat[2][0] = cos(lat) * cos(lon);
  trans_mat[2][1] = cos(lat) * sin(lon);
  trans_mat[2][2] = sin(lat);

  Vector<3> sc_pos_ltc = trans_mat * (sc_pos_ecef - gs_pos_ecef);  // Satellite position in LTC frame [m]
  Vector<3> dir_GS_to_zenith = Vector<3>(0);
  dir_GS_to_zenith[2] = 1;

  // Judge the satellite position angle is over the minimum elevation
  if (dot(sc_pos_ltc, dir_GS_to_zenith) > norm(sc_pos_ltc) * sin(groundstation.GetElevationLimitAngle_deg() * libra::deg_to_rad)) {
    return true;
  } else {
    return false;
  }
}

double GScalculator::CalcMaxBitrate(const Dynamics& dynamics, const Antenna& sc_ant, const GroundStation& groundstation, const Antenna& gs_ant) {
  if (!sc_ant.IsTransmitter() || !gs_ant.IsReceiver()) {
    // Check compatibility of transmitter and receiver
    return 0.0f;
  }

  // TODO: Consider Inertial frame of ECEF frame?
  Vector<3> sc_pos_i = dynamics.GetOrbit().GetSatPosition_i();
  Vector<3> gs_pos_i = groundstation.GetGSPosition_i();
  double dist_sc_gs_km = norm(sc_pos_i - gs_pos_i) / 1000;
  double loss_space_dB = -20 * log10(4 * libra::pi * dist_sc_gs_km / (300 / sc_ant.GetFrequency() / 1000));

  // TODO: Calculate the boresight angle from position of the satellite
  double sc_boresight_angle_rad = 0;
  // Ref.  // double theta = angle(q_b2i.frame_conv(axis_b), rel_pos);

  double gs_boresight_angle_rad = 0;  // Assume the GS antenna completely track the satellite

  double cn0_dBHz = sc_ant.CalcTxEIRP(sc_boresight_angle_rad) + loss_space_dB + loss_polarization_ + loss_atmosphere_ + loss_rainfall_ +
                    loss_others_ + gs_ant.CalcRxGT(gs_boresight_angle_rad) - 10 * log10(environment::boltzmann_constant_J_K);

  double margin_for_bitrate_dB = cn0_dBHz - (EbN0_ + hardware_deterioration_ + coding_gain_) - margin_req_;

  if (margin_for_bitrate_dB > 0) {
    return pow(10, margin_for_bitrate_dB / 10.) / 1000000.;  //[MHz]? TODO: Need to check the correctness...
  } else {
    return 0.0;
  }
}

std::string GScalculator::GetLogHeader() const {
  std::string str_tmp = "";

  str_tmp += WriteScalar("is visible");
  str_tmp += WriteScalar("max bitrate[Mbps]");

  return str_tmp;
}

std::string GScalculator::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteScalar(is_visible_);
  str_tmp += WriteScalar(max_bitrate_);

  return str_tmp;
}
