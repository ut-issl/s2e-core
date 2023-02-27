/*
 * @file antenna.cpp
 * @brief Component emulation: RF antenna
 */

#include "antenna.hpp"

#include <cmath>
#include <library/utilities/macros.hpp>

Antenna::Antenna(const int id, const libra::Quaternion& quaternion_b2c, const bool is_transmitter, const bool is_receiver, const double frequency,
                 const Vector<4> tx_params, const Vector<4> rx_params)
    : id_(id), is_transmitter_(is_transmitter), is_receiver_(is_receiver), frequency_(frequency) {
  quaternion_b2c_ = quaternion_b2c;

  // Parameters
  tx_output_power_W_ = tx_params[0];
  tx_params_.gain_dBi_ = tx_params[1];
  tx_params_.loss_feeder_dB_ = tx_params[2];
  tx_params_.loss_pointing_dB_ = tx_params[3];

  rx_params_.gain_dBi_ = rx_params[0];
  rx_params_.loss_feeder_dB_ = rx_params[1];
  rx_params_.loss_pointing_dB_ = rx_params[2];
  rx_system_noise_temperature_K_ = rx_params[3];

  // Antenna gain
  tx_params_.antenna_gain_model = AntennaGainModel::ISOTROPIC;
  rx_params_.antenna_gain_model = AntennaGainModel::ISOTROPIC;

  // Calculate the EIRP or GT for the maximum gain
  if (is_transmitter_) {
    tx_eirp_dBW_ = 10 * log10(tx_output_power_W_) + tx_params_.loss_feeder_dB_ + tx_params_.loss_pointing_dB_;
  } else {
    tx_eirp_dBW_ = 0.0;
  }
  if (is_receiver_) {
    rx_gt_dBK_ = rx_params_.loss_feeder_dB_ + rx_params_.loss_pointing_dB_ - 10 * std::log10(rx_system_noise_temperature_K_);
  } else {
    rx_gt_dBK_ = 0.0;
  }
}

Antenna::Antenna(const int id, const libra::Quaternion& quaternion_b2c, const bool is_transmitter, const bool is_receiver, const double frequency,
                 const double tx_output_power_W, const AntennaParameters tx_params, const double rx_system_noise_temperature_K,
                 const AntennaParameters rx_params)
    : id_(id),
      quaternion_b2c_(quaternion_b2c),
      is_transmitter_(is_transmitter),
      is_receiver_(is_receiver),
      frequency_(frequency),
      tx_output_power_W_(tx_output_power_W),
      tx_params_(tx_params),
      rx_system_noise_temperature_K_(rx_system_noise_temperature_K),
      rx_params_(rx_params) {
  // Calculate the EIRP or GT for the maximum gain
  if (is_transmitter_) {
    tx_eirp_dBW_ = 10 * log10(tx_output_power_W_) + tx_params_.loss_feeder_dB_ + tx_params_.loss_pointing_dB_;
  } else {
    tx_eirp_dBW_ = 0.0;
  }
  if (is_receiver_) {
    rx_gt_dBK_ = rx_params_.loss_feeder_dB_ + rx_params_.loss_pointing_dB_ - 10 * std::log10(rx_system_noise_temperature_K_);
  } else {
    rx_gt_dBK_ = 0.0;
  }
}

Antenna::~Antenna() {}

double Antenna::CalcAntennaGain(const AntennaParameters ant_params, const double theta_rad, const double phi_rad) const {
  double gain_dBi = 0.0;
  switch (ant_params.antenna_gain_model) {
    case AntennaGainModel::ISOTROPIC:
      gain_dBi = ant_params.gain_dBi_;
      break;
    case AntennaGainModel::RADIATION_PATTERN_CSV:
      gain_dBi = ant_params.radiation_pattern.GetGain_dBi(theta_rad, phi_rad);
      break;
    default:
      break;
  }
  return gain_dBi;
}

double Antenna::CalcTxEIRP(const double theta_rad, const double phi_rad) const {
  return tx_eirp_dBW_ + CalcAntennaGain(tx_params_, theta_rad, phi_rad);
}
double Antenna::CalcRxGT(const double theta_rad, const double phi_rad) const { return rx_gt_dBK_ + CalcAntennaGain(rx_params_, theta_rad, phi_rad); }

AntennaGainModel SetAntennaGainModel(const std::string gain_model_name) {
  if (gain_model_name == "ISOTROPIC") {
    return AntennaGainModel::ISOTROPIC;
  } else if (gain_model_name == "RADIATION_PATTERN_CSV") {
    return AntennaGainModel::RADIATION_PATTERN_CSV;
  } else {
    return AntennaGainModel::ISOTROPIC;
  }
}
