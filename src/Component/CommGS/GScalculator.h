/*
* @file GScalculator.h
* @brief 地上局クラスが持つ，通信関連計算模擬クラスです．
* @author 山本 智貴
* @date 2020.05.26
*/

#pragma once
#include <Interface/LogOutput/ILoggable.h>
#include <Component/CommGS/ANT.h>
#include <Dynamics/Dynamics.h>
#include <Simulation/GroundStation/GroundStation.h>
#include <Library/math/Matrix.hpp>
#include <Library/math/Vector.hpp>
#include <Library/math/MatVec.hpp>

using libra::Matrix;
using libra::Vector;

//↓TODO: 地上局位置をECI2ECEF変換するためにユリウス時が必要なのでDynamicsに作ったGetCurrentJd()で持ち出して直接SGPをいじっているが，これをDynamics外に出すissueがあるので，いずれそれと関連して間接的にいじるように変える必要がある
// https://gitlab.com/ut_issl/s2e/s2e_core_oss/-/issues/4
#include <Library/sgp4/sgp4unit.h>
#include <Library/sgp4/sgp4io.h>
#include <Library/sgp4/sgp4ext.h>
// using namespace std;
// #define DEG2RAD 0.017453292519943295769  // PI/180
// static gravconsttype whichconst;
//↑


class GScalculator : public ILoggable
{
public:
    double loss_polarization_;  //[dB]
    double loss_atmosphere_;  //[dB]
    double loss_rainfall_;  //[dB]
    double loss_others_;  //[dB]
    double EbN0_;  //[dB]
    double hardware_deterioration_;  //[dB]
    double coding_gain_;  //[dB]
    double margin_req_;  //[dB]

    bool visible_flag_;
    double max_bitrate_;  //[kbps]

    GScalculator(double loss_polarization,
                 double loss_atmosphere,
                 double loss_rainfall,
                 double loss_others,
                 double EbN0,
                 double hardware_deterioration,
                 double coding_gain,
                 double margin_req);
    ~GScalculator();
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
