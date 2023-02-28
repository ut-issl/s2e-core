/*
 * @file antenna.cpp
 * @brief Component emulation: RF antenna
 */

#include "antenna.hpp"

#include <cmath>
#include <library/utilities/macros.hpp>

Antenna::Antenna(const int component_id, const libra::Quaternion& quaternion_b2c, const bool is_transmitter, const bool is_receiver,
                 const double frequency_MHz, const Vector<4> tx_parameters, const Vector<4> rx_parameters)
    : component_id_(component_id), is_transmitter_(is_transmitter), is_receiver_(is_receiver), frequency_MHz_(frequency_MHz) {
  quaternion_b2c_ = quaternion_b2c;

  // Parameters
  tx_output_power_W_ = tx_parameters[0];
  tx_parameters_.gain_dBi_ = tx_parameters[1];
  tx_parameters_.loss_feeder_dB_ = tx_parameters[2];
  tx_parameters_.loss_pointing_dB_ = tx_parameters[3];

  rx_parameters_.gain_dBi_ = rx_parameters[0];
  rx_parameters_.loss_feeder_dB_ = rx_parameters[1];
  rx_parameters_.loss_pointing_dB_ = rx_parameters[2];
  rx_system_noise_temperature_K_ = rx_parameters[3];

  // Antenna gain
  tx_parameters_.antenna_gain_model = AntennaGainModel::ISOTROPIC;
  rx_parameters_.antenna_gain_model = AntennaGainModel::ISOTROPIC;

  // Calculate the EIRP or GT for the maximum gain
  if (is_transmitter_) {
    tx_eirp_dBW_ = 10 * log10(tx_output_power_W_) + tx_parameters_.loss_feeder_dB_ + tx_parameters_.loss_pointing_dB_;
  } else {
    tx_eirp_dBW_ = 0.0;
  }
  if (is_receiver_) {
    rx_gt_dBK_ = rx_parameters_.loss_feeder_dB_ + rx_parameters_.loss_pointing_dB_ - 10 * std::log10(rx_system_noise_temperature_K_);
  } else {
    rx_gt_dBK_ = 0.0;
  }
}

Antenna::Antenna(const int component_id, const libra::Quaternion& quaternion_b2c, const bool is_transmitter, const bool is_receiver,
                 const double frequency_MHz, const double tx_output_power_W, const AntennaParameters tx_parameters,
                 const double rx_system_noise_temperature_K, const AntennaParameters rx_parameters)
    : component_id_(component_id),
      quaternion_b2c_(quaternion_b2c),
      is_transmitter_(is_transmitter),
      is_receiver_(is_receiver),
      frequency_MHz_(frequency_MHz),
      tx_output_power_W_(tx_output_power_W),
      tx_parameters_(tx_parameters),
      rx_system_noise_temperature_K_(rx_system_noise_temperature_K),
      rx_parameters_(rx_parameters) {
  // Calculate the EIRP or GT for the maximum gain
  if (is_transmitter_) {
    tx_eirp_dBW_ = 10 * log10(tx_output_power_W_) + tx_parameters_.loss_feeder_dB_ + tx_parameters_.loss_pointing_dB_;
  } else {
    tx_eirp_dBW_ = 0.0;
  }
  if (is_receiver_) {
    rx_gt_dBK_ = rx_parameters_.loss_feeder_dB_ + rx_parameters_.loss_pointing_dB_ - 10 * std::log10(rx_system_noise_temperature_K_);
  } else {
    rx_gt_dBK_ = 0.0;
  }
}

Antenna::~Antenna() {}

double Antenna::CalcAntennaGain(const AntennaParameters antenna_parameters, const double theta_rad, const double phi_rad) const {
  double gain_dBi = 0.0;
  switch (antenna_parameters.antenna_gain_model) {
    case AntennaGainModel::ISOTROPIC:
      gain_dBi = antenna_parameters.gain_dBi_;
      break;
    case AntennaGainModel::RADIATION_PATTERN_CSV:
      gain_dBi = antenna_parameters.radiation_pattern.GetGain_dBi(theta_rad, phi_rad);
      break;
    default:
      break;
  }
  return gain_dBi;
}

double Antenna::CalcTxEIRP_dBW(const double theta_rad, const double phi_rad) const {
  return tx_eirp_dBW_ + CalcAntennaGain(tx_parameters_, theta_rad, phi_rad);
}
double Antenna::CalcRxGT_dB_K(const double theta_rad, const double phi_rad) const {
  return rx_gt_dBK_ + CalcAntennaGain(rx_parameters_, theta_rad, phi_rad);
}

AntennaGainModel SetAntennaGainModel(const std::string gain_model_name) {
  if (gain_model_name == "ISOTROPIC") {
    return AntennaGainModel::ISOTROPIC;
  } else if (gain_model_name == "RADIATION_PATTERN_CSV") {
    return AntennaGainModel::RADIATION_PATTERN_CSV;
  } else {
    return AntennaGainModel::ISOTROPIC;
  }
}
