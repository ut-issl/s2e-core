/*
 * @file GScalculator.cpp
 * @brief 地上局クラスが持つ，通信関連計算模擬クラスです．
 * @author 山本 智貴
 * @date 2020.05.26
 */

#define _USE_MATH_DEFINES
#include <math.h>

#include "GScalculator.h"
#define Kb 1.38064852E-23               // Boltzmann constant
#define DEG2RAD 0.017453292519943295769 // PI/180

GScalculator::GScalculator(double loss_polarization, double loss_atmosphere,
                           double loss_rainfall, double loss_others,
                           double EbN0, double hardware_deterioration,
                           double coding_gain, double margin_req)
    : loss_polarization_(loss_polarization), loss_atmosphere_(loss_atmosphere),
      loss_rainfall_(loss_rainfall), loss_others_(loss_others), EbN0_(EbN0),
      hardware_deterioration_(hardware_deterioration),
      coding_gain_(coding_gain), margin_req_(margin_req) {
  visible_flag_ = false;
  max_bitrate_ = 0.0f;
}

GScalculator::~GScalculator() {}

void GScalculator::Initialize() {}

void GScalculator::Update(const Dynamics &dynamics, const ANT &sc_ant,
                          const GroundStation &groundstation,
                          const ANT &gs_ant) {
  visible_flag_ = IsVisible(dynamics, groundstation);
  if (visible_flag_) {
    max_bitrate_ = CalcMaxBitrate(dynamics, sc_ant, groundstation, gs_ant);
  } else {
    max_bitrate_ = 0.0f;
  }
}

bool GScalculator::IsVisible(const Dynamics &dynamics,
                             const GroundStation &groundstation) {
  Vector<3> sc_pos_ecef = dynamics.GetOrbit().GetSatPosition_ecef();

  Vector<3> gs_pos_i = groundstation.GetGSPosition_i();
  Matrix<3, 3> DCM_ecei_ecef = dynamics.GetOrbit().GetTransECItoECEF();
  Vector<3> gs_pos_ecef = DCM_ecei_ecef * gs_pos_i;

  double lat = groundstation.latitude_ * DEG2RAD;  //[rad]
  double lon = groundstation.longitude_ * DEG2RAD; //[rad]

  Matrix<3, 3> trans_mat; // 地上局におけるECEF2LTC変換行列の回転部分
  trans_mat[0][0] = -sin(lon);
  trans_mat[0][1] = cos(lon);
  trans_mat[0][2] = 0;
  trans_mat[1][0] = -sin(lat) * cos(lon);
  trans_mat[1][1] = -sin(lat) * sin(lon);
  trans_mat[1][2] = cos(lat);
  trans_mat[2][0] = cos(lat) * cos(lon);
  trans_mat[2][1] = cos(lat) * sin(lon);
  trans_mat[2][2] = sin(lat);

  Vector<3> sc_pos_ltc =
      trans_mat * (sc_pos_ecef - gs_pos_ecef); //局所座標系における衛星位置

  // 地上局から天頂への単位ベクトル
  Vector<3> dir_GS_to_zenith = Vector<3>(0);
  dir_GS_to_zenith[2] = 1;

  // 地上局の最低可視仰角をクリアしているかを判定
  if (dot(sc_pos_ltc, dir_GS_to_zenith) >
      norm(sc_pos_ltc) * sin(groundstation.elevation_angle_ * DEG2RAD)) {
    // std::cout << std::asin(dot(sc_pos_ltc, dir_GS_to_zenith) /
    // norm(sc_pos_ltc)) / DEG2RAD << std::endl;
    return true;
  } else {
    return false;
  }
}

double GScalculator::CalcMaxBitrate(const Dynamics &dynamics, const ANT &sc_ant,
                                    const GroundStation &groundstation,
                                    const ANT &gs_ant) {
  if (!sc_ant.is_transmitter_ || !gs_ant.is_receiver_) {
    return 0.0f; //送受信の噛み合わせをここでチェック（いずれどのidのANTを使うかとDLとULどっちにするかを指定できるようにしないといけない）
  }

  Vector<3> sc_pos_i = dynamics.GetOrbit().GetSatPosition_i();
  Vector<3> gs_pos_i = groundstation.GetGSPosition_i();
  double dist_sc_gs = norm(sc_pos_i - gs_pos_i) / 1000; //[km]
  double loss_space = -20 * log10(4 * M_PI * dist_sc_gs /
                                  (300 / sc_ant.frequency_ / 1000)); //[dB]

  double sc_boresight_angle =
      0; // 衛星姿勢と地上局との位置関係から，電波方向のボアサイトからの角度を求める（今はANT::CalcAntennaGain()も未実装のため0と適当に置いておく）
  //参考  // double theta = angle(q_b2i.frame_conv(axis_b), rel_pos);
  double gs_boresight_angle =
      0; // 地上局アンテナは追尾を行うとして，最大ゲインを適用できると考える

  double CN0 = sc_ant.GetTxEIRP(sc_boresight_angle) + loss_space +
               loss_polarization_ + loss_atmosphere_ + loss_rainfall_ +
               loss_others_ + gs_ant.GetRxGT(gs_boresight_angle) -
               10 * log10(Kb); //[dBHz]

  double margin_for_bitrate = CN0 -
                              (EbN0_ + hardware_deterioration_ + coding_gain_) -
                              margin_req_; //[dB]

  if (margin_for_bitrate > 0) {
    return pow(10, margin_for_bitrate / 10.) / 1000000.; //[MHz]
  } else {
    return 0.0;
  }
}

std::string GScalculator::GetLogHeader() const {
  std::string str_tmp = "";

  str_tmp += WriteScalar("visible flag");
  str_tmp += WriteScalar("max bitrate[Mbps]");

  return str_tmp;
}

std::string GScalculator::GetLogValue() const {
  std::string str_tmp = "";

  str_tmp += WriteScalar(visible_flag_);
  str_tmp += WriteScalar(max_bitrate_);

  return str_tmp;
}
