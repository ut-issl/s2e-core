/*
 * @file Antenna.cpp
 * @brief Component emuration: RF antenna
 * @author 山本 智貴
 */

#include "Antenna.hpp"

#include <Library/utils/Macros.hpp>
#include <cmath>

Antenna::Antenna(const int id, const libra::Quaternion& q_b2c, const bool is_transmitter, const bool is_receiver, const double frequency,
                 const Vector<4> tx_params, const Vector<4> rx_params)
    : id_(id), is_transmitter_(is_transmitter), is_receiver_(is_receiver), frequency_(frequency) {
  q_b2c_ = q_b2c;
  tx_output_ = tx_params[0];
  tx_gain_ = tx_params[1];
  tx_loss_feeder_ = tx_params[2];
  tx_loss_pointing_ = tx_params[3];
  rx_gain_ = rx_params[0];
  rx_loss_feeder_ = rx_params[1];
  rx_loss_pointing_ = rx_params[2];
  rx_system_noise_temperature_ = rx_params[3];

  // Calculate the EIRP or GT for the maximum gain
  if (is_transmitter_) {
    tx_EIRP_ = 10 * log10(tx_output_) + tx_gain_ + tx_loss_feeder_ + tx_loss_pointing_;
  } else {
    tx_EIRP_ = 0.0;
  }
  if (is_receiver_) {
    rx_GT_ = rx_gain_ + rx_loss_feeder_ + rx_loss_pointing_ - 10 * std::log10(rx_system_noise_temperature_);
  } else {
    rx_GT_ = 0.0;
  }
}

Antenna::~Antenna() {}

double Antenna::CalcAntennaGain(double theta, bool is_tx) const {
  UNUSED(theta);
  // TODO: implement gain calculation considering the angle theta

  if (is_tx) {
    return tx_gain_;
  } else {
    return rx_gain_;
  }
}

double Antenna::CalcTxEIRP(double theta) const { return tx_EIRP_ - tx_gain_ + CalcAntennaGain(theta, true); }
double Antenna::CalcRxGT(double theta) const { return rx_GT_ - rx_gain_ + CalcAntennaGain(theta, false); }
