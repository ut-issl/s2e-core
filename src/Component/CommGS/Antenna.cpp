/*
 * @file Antenna.cpp
 * @brief Component emulation: RF antenna
 */

#include "Antenna.hpp"

#include <Library/utils/Macros.hpp>
#include <cmath>

Antenna::Antenna(const int id, const libra::Quaternion& q_b2c, const bool is_transmitter, const bool is_receiver, const double frequency,
                 const Vector<4> tx_params, const Vector<4> rx_params)
    : id_(id), is_transmitter_(is_transmitter), is_receiver_(is_receiver), frequency_(frequency) {
  q_b2c_ = q_b2c;

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
    tx_eirp_dBW_ = 10 * log10(tx_output_power_W_) + tx_params_.gain_dBi_ + tx_params_.loss_feeder_dB_ + tx_params_.loss_pointing_dB_;
  } else {
    tx_eirp_dBW_ = 0.0;
  }
  if (is_receiver_) {
    rx_gt_dBK_ = rx_params_.gain_dBi_ + rx_params_.loss_feeder_dB_ + rx_params_.loss_pointing_dB_ - 10 * std::log10(rx_system_noise_temperature_K_);
  } else {
    rx_gt_dBK_ = 0.0;
  }
}

Antenna::Antenna(const int id, const libra::Quaternion& q_b2c, const bool is_transmitter, const bool is_receiver, const double frequency,
          const double tx_output_power_W, const AntennaParameters tx_params, const double rx_system_noise_temperature_K, const AntennaParameters rx_params)
    : id_(id), q_b2c_(q_b2c), is_transmitter_(is_transmitter), is_receiver_(is_receiver), frequency_(frequency), tx_output_power_W_(tx_output_power_W), tx_params_(tx_params), rx_system_noise_temperature_K_(rx_system_noise_temperature_K), rx_params_(rx_params)
{
  // Calculate the EIRP or GT for the maximum gain
  if (is_transmitter_) {
    tx_eirp_dBW_ = 10 * log10(tx_output_power_W_) + tx_params_.gain_dBi_ + tx_params_.loss_feeder_dB_ + tx_params_.loss_pointing_dB_;
  } else {
    tx_eirp_dBW_ = 0.0;
  }
  if (is_receiver_) {
    rx_gt_dBK_ = rx_params_.gain_dBi_ + rx_params_.loss_feeder_dB_ + rx_params_.loss_pointing_dB_ - 10 * std::log10(rx_system_noise_temperature_K_);
  } else {
    rx_gt_dBK_ = 0.0;
  }
}

Antenna::~Antenna() {}

double Antenna::CalcAntennaGain(const bool is_tx, const double theta_rad, const double phi_rad) const {
  // TODO: implement gain calculation considering the angle theta

  if (is_tx) {
    return tx_params_.gain_dBi_;
  } else {
    return rx_params_.gain_dBi_;
  }
}

double Antenna::CalcTxEIRP(const double theta_rad, const double phi_rad) const { return tx_eirp_dBW_ - tx_params_.gain_dBi_ + CalcAntennaGain(true, theta_rad, phi_rad); }
double Antenna::CalcRxGT(const double theta_rad, const double phi_rad) const { return rx_gt_dBK_ - rx_params_.gain_dBi_ + CalcAntennaGain(false, theta_rad, phi_rad); }
