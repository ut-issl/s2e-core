/*
 * @file ground_station_calculator.cpp
 * @brief Emulation of analysis and calculation for Ground Stations
 */

#include "ground_station_calculator.hpp"

#include <environment/global/physical_constants.hpp>
#include <library/initialize/initialize_file_access.hpp>
#include <library/math/constants.hpp>

GroundStationCalculator::GroundStationCalculator(const double loss_polarization_dB, const double loss_atmosphere_dB, const double loss_rainfall_dB,
                                                 const double loss_others_dB, const double ebn0_dB, const double hardware_deterioration_dB,
                                                 const double coding_gain_dB, const double margin_requirement_dB, const double downlink_bitrate_bps)
    : loss_polarization_dB_(loss_polarization_dB),
      loss_atmosphere_dB_(loss_atmosphere_dB),
      loss_rainfall_dB_(loss_rainfall_dB),
      loss_others_dB_(loss_others_dB),
      ebn0_dB_(ebn0_dB),
      hardware_deterioration_dB_(hardware_deterioration_dB),
      coding_gain_dB_(coding_gain_dB),
      margin_requirement_dB_(margin_requirement_dB),
      downlink_bitrate_bps_(downlink_bitrate_bps) {
  max_bitrate_Mbps_ = 0.0;
  receive_margin_dB_ = -10000.0;  // FIXME: which value is suitable?
}

GroundStationCalculator::~GroundStationCalculator() {}

void GroundStationCalculator::Update(const Spacecraft& spacecraft, const Antenna& spacecraft_tx_antenna, const GroundStation& ground_station,
                                     const Antenna& ground_station_rx_antenna) {
  bool is_visible = ground_station.IsVisible(spacecraft.GetSpacecraftId());
  if (is_visible) {
    max_bitrate_Mbps_ = CalcMaxBitrate(spacecraft.GetDynamics(), spacecraft_tx_antenna, ground_station, ground_station_rx_antenna);
    receive_margin_dB_ = CalcReceiveMarginOnGs(spacecraft.GetDynamics(), spacecraft_tx_antenna, ground_station, ground_station_rx_antenna);
  } else {
    max_bitrate_Mbps_ = 0.0;
    receive_margin_dB_ = -10000.0;  // FIXME: which value is suitable?
  }
}

// Private functions
double GroundStationCalculator::CalcMaxBitrate(const Dynamics& dynamics, const Antenna& spacecraft_tx_antenna, const GroundStation& ground_station,
                                               const Antenna& ground_station_rx_antenna) {
  double cn0_dBHz = CalcCn0OnGs(dynamics, spacecraft_tx_antenna, ground_station, ground_station_rx_antenna);

  double margin_for_bitrate_dB = cn0_dBHz - (ebn0_dB_ + hardware_deterioration_dB_ + coding_gain_dB_) - margin_requirement_dB_;

  if (margin_for_bitrate_dB > 0) {
    return pow(10.0, margin_for_bitrate_dB / 10.0) / 1000000.0;
  } else {
    return 0.0;
  }
}

double GroundStationCalculator::CalcReceiveMarginOnGs(const Dynamics& dynamics, const Antenna& spacecraft_tx_antenna,
                                                      const GroundStation& ground_station, const Antenna& ground_station_rx_antenna) {
  double cn0_dB = CalcCn0OnGs(dynamics, spacecraft_tx_antenna, ground_station, ground_station_rx_antenna);
  double cn0_requirement_dB = ebn0_dB_ + hardware_deterioration_dB_ + coding_gain_dB_ + 10.0 * log10(spacecraft_tx_antenna.GetBitrate_bps());
  return cn0_dB - cn0_requirement_dB;
}

double GroundStationCalculator::CalcCn0OnGs(const Dynamics& dynamics, const Antenna& spacecraft_tx_antenna, const GroundStation& ground_station,
                                            const Antenna& ground_station_rx_antenna) {
  if (!spacecraft_tx_antenna.IsTransmitter() || !ground_station_rx_antenna.IsReceiver()) {
    // Check compatibility of transmitter and receiver
    return 0.0f;
  }
  // Free space path loss
  Vector<3> sc_pos_i = dynamics.GetOrbit().GetPosition_i_m();
  Vector<3> gs_pos_i = ground_station.GetPosition_i_m();
  Vector<3> pos_gs2sc_i = sc_pos_i - gs_pos_i;

  double dist_sc_gs_km = pos_gs2sc_i.CalcNorm() / 1000.0;
  double loss_space_dB = -20.0 * log10(4.0 * libra::pi * dist_sc_gs_km / (300.0 / spacecraft_tx_antenna.GetFrequency_MHz() / 1000.0));

  // GS direction on SC TX antenna frame
  Vector<3> sc_to_gs_i = gs_pos_i - sc_pos_i;
  sc_to_gs_i = sc_to_gs_i.CalcNormalizedVector();
  Quaternion q_i_to_sc_ant = spacecraft_tx_antenna.GetQuaternion_b2c() * dynamics.GetAttitude().GetQuaternion_i2b();
  Vector<3> gs_direction_on_sc_frame = q_i_to_sc_ant.FrameConversion(sc_to_gs_i);
  double theta_on_sc_antenna_rad = acos(gs_direction_on_sc_frame[2]);
  double phi_on_sc_antenna_rad = atan2(gs_direction_on_sc_frame[1], gs_direction_on_sc_frame[0]);

  // SC direction on GS RX antenna frame
  Vector<3> gs_to_sc_ecef = dynamics.GetOrbit().GetPosition_ecef_m() - ground_station.GetPosition_ecef_m();
  gs_to_sc_ecef = gs_to_sc_ecef.CalcNormalizedVector();
  Quaternion q_ecef_to_gs_ant = ground_station_rx_antenna.GetQuaternion_b2c() * ground_station.GetGeodeticPosition().GetQuaternionXcxfToLtc();
  Vector<3> sc_direction_on_gs_frame = q_ecef_to_gs_ant.FrameConversion(gs_to_sc_ecef);
  double theta_on_gs_antenna_rad = acos(sc_direction_on_gs_frame[2]);
  double phi_on_gs_antenna_rad = atan2(sc_direction_on_gs_frame[1], sc_direction_on_gs_frame[0]);

  // Calc CN0
  double cn0_dBHz = spacecraft_tx_antenna.CalcTxEirp_dBW(theta_on_sc_antenna_rad, phi_on_sc_antenna_rad) + loss_space_dB + loss_polarization_dB_ +
                    loss_atmosphere_dB_ + loss_rainfall_dB_ + loss_others_dB_ +
                    ground_station_rx_antenna.CalcRxGt_dB_K(theta_on_gs_antenna_rad, phi_on_gs_antenna_rad) -
                    10.0 * log10(environment::boltzmann_constant_J_K);
  return cn0_dBHz;
}

std::string GroundStationCalculator::GetLogHeader() const {
  std::string str_tmp = "";
  std::string component_name = "gs_calculator_";

  str_tmp += WriteScalar(component_name + "max_bitrate", "Mbps");
  str_tmp += WriteScalar(component_name + "receive_margin", "dB");

  return str_tmp;
}

std::string GroundStationCalculator::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteScalar(max_bitrate_Mbps_);
  str_tmp += WriteScalar(receive_margin_dB_);

  return str_tmp;
}

GroundStationCalculator InitGsCalculator(const std::string file_name) {
  IniAccess gs_conf(file_name);

  char Section[30] = "GROUND_STATION_CALCULATOR";

  double loss_polarization_dB = gs_conf.ReadDouble(Section, "loss_polarization_dB");
  double loss_atmosphere_dB = gs_conf.ReadDouble(Section, "loss_atmosphere_dB");
  double loss_rainfall_dB = gs_conf.ReadDouble(Section, "loss_rainfall_dB");
  double loss_others_dB = gs_conf.ReadDouble(Section, "loss_others_dB");
  double ebn0_dB = gs_conf.ReadDouble(Section, "ebn0_dB");
  double hardware_deterioration_dB = gs_conf.ReadDouble(Section, "hardware_deterioration_dB");
  double coding_gain_dB = gs_conf.ReadDouble(Section, "coding_gain_dB");
  double margin_requirement_dB = gs_conf.ReadDouble(Section, "margin_requirement_dB");
  double downlink_bitrate_bps = gs_conf.ReadDouble(Section, "downlink_bitrate_bps");

  GroundStationCalculator gs_calculator(loss_polarization_dB, loss_atmosphere_dB, loss_rainfall_dB, loss_others_dB, ebn0_dB,
                                        hardware_deterioration_dB, coding_gain_dB, margin_requirement_dB, downlink_bitrate_bps);
  return gs_calculator;
}
