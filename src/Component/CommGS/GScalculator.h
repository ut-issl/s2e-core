/*
 * @file GScalculator.h
 * @brief 地上局クラスが持つ，通信関連計算模擬クラスです．
 * @author 山本 智貴
 * @date 2020.05.26
 */

#pragma once
#include <Dynamics/Dynamics.h>
#include <Environment/Global/GlobalEnvironment.h>
#include <Interface/LogOutput/ILoggable.h>
#include <Simulation/GroundStation/GroundStation.h>

#include <Component/CommGS/Antenna.hpp>
#include <Library/math/MatVec.hpp>
#include <Library/math/Matrix.hpp>
#include <Library/math/Vector.hpp>

using libra::Matrix;
using libra::Vector;

class GScalculator : public ILoggable {
 public:
  double loss_polarization_;       //[dB]
  double loss_atmosphere_;         //[dB]
  double loss_rainfall_;           //[dB]
  double loss_others_;             //[dB]
  double EbN0_;                    //[dB]
  double hardware_deterioration_;  //[dB]
  double coding_gain_;             //[dB]
  double margin_req_;              //[dB]

  bool visible_flag_;
  double max_bitrate_;  //[kbps]

  GScalculator(double loss_polarization, double loss_atmosphere, double loss_rainfall, double loss_others, double EbN0, double hardware_deterioration,
               double coding_gain, double margin_req);
  virtual ~GScalculator();
  void Initialize();
  void Update(const Dynamics& dynamics, const ANT& sc_ant, const GroundStation& groundstation, const ANT& gs_ant);

  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;

 protected:
  // 衛星と地上局が可視の位置関係のときにtrue，非可視のときにfalseを返す
  bool IsVisible(const Dynamics& dynamics, const GroundStation& groundstation);

  // 最大可能ビットレートを回線計算をもとに計算する
  double CalcMaxBitrate(const Dynamics& dynamics, const ANT& sc_ant, const GroundStation& groundstation, const ANT& gs_ant);
};
