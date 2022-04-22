/*
 * @file ANT.cpp
 * @brief アンテナ模擬コンポです．
 * @author 山本 智貴
 */

#include "ANT.h"

#include <Library/utils/Unused.hpp>
#include <cmath>

ANT::ANT(int ant_id, const libra::Quaternion& q_b2c, bool is_transmitter, bool is_receiver, double frequency, Vector<4> tx_params,
         Vector<4> rx_params)
    : ant_id_(ant_id), is_transmitter_(is_transmitter), is_receiver_(is_receiver), frequency_(frequency) {
  q_b2c_ = q_b2c;
  tx_output_ = tx_params[0];
  tx_gain_ = tx_params[1];
  tx_loss_feeder_ = tx_params[2];
  tx_loss_pointing_ = tx_params[3];
  rx_gain_ = rx_params[0];
  rx_loss_feeder_ = rx_params[1];
  rx_loss_pointing_ = rx_params[2];
  rx_system_noise_temperature_ = rx_params[3];

  if (is_transmitter_) {
    tx_EIRP_ = 10 * log10(tx_output_) + tx_gain_ + tx_loss_feeder_ + tx_loss_pointing_;  // 初期化では最大ゲインの場合で計算しておく
  } else {
    tx_EIRP_ = 0.0;
  }
  if (is_receiver_) {
    rx_GT_ =
        rx_gain_ + rx_loss_feeder_ + rx_loss_pointing_ - 10 * std::log10(rx_system_noise_temperature_);  // 初期化では最大ゲインの場合で計算しておく
  } else {
    rx_GT_ = 0.0;
  }
}

ANT::~ANT() {}

void ANT::Initialize() {}

double ANT::CalcAntennaGain(double theta, bool is_tx) const  // アンテナパターンをもとに姿勢方向に応じてゲインを返したい
{
  UNUSED(theta);  // TODO: use this parameter

  if (is_tx) {
    return tx_gain_;
  } else {
    return rx_gain_;
  }
}

double ANT::GetTxEIRP(double theta) const { return tx_EIRP_ - tx_gain_ + CalcAntennaGain(theta, true); }
double ANT::GetRxGT(double theta) const { return rx_GT_ - rx_gain_ + CalcAntennaGain(theta, false); }
