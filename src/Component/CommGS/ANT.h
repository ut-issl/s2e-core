/*
 * @file ANT.cpp
 * @brief アンテナ模擬コンポです．
 * @author 山本 智貴
 * @date 2020.05.16
 */

#pragma once
#include "../../Library/math/Quaternion.hpp"
#include "../../Library/math/Vector.hpp"
using libra::Quaternion;
using libra::Vector;

class ANT {
public:
  int ant_id_;          // ANTのID
  Quaternion q_b2c_;    // ANT搭載面
  bool is_transmitter_; //送信用ANTか否か
  bool is_receiver_;    //受信用ANTか否か
  double frequency_;    //中心周波数[MHz]

  ANT(int ant_id, const libra::Quaternion &q_b2c, bool is_transmitter,
      bool is_receiver, double frequency, Vector<4> tx_params,
      Vector<4> rx_params);
  ~ANT();
  void Initialize();
  double GetTxEIRP(double theta) const;
  double GetRxGT(double theta) const;

protected:
  double tx_output_; // RF出力[W]
  double tx_gain_; //送信ゲイン[dBi]（「最大ゲイン」が適切？）
  double tx_loss_feeder_;   //給電損失[dB]
  double tx_loss_pointing_; //ポインティング損失[dB]
  double rx_gain_; //受信ゲイン[dBi]（「最大ゲイン」が適切？）
  double rx_loss_feeder_;              //給電損失[dB]
  double rx_loss_pointing_;            //ポインティング損失[dB]
  double rx_system_noise_temperature_; //システム雑音温度[K]

  double tx_EIRP_; //送信EIRP[dBW]
  double rx_GT_;   //受信G/T[dB/K]

  double CalcAntennaGain(double theta, bool is_tx)
      const; //方向を指定したときのアンテナゲイン[dB]を返す
};
