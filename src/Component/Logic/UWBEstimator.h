#pragma once

#include <iterator>
#include <vector>

#include "../../Library/math/MatVec.hpp"
#include "../../Library/math/Quaternion.hpp"
using libra::Matrix;
using libra::Quaternion;
using libra::Vector;

#include "../../Interface/LogOutput/ILoggable.h"

class UWBSensor;
using uwbvec = std::vector<UWBSensor *>;

class UWBEstimator : public ILoggable {
public:
  UWBEstimator(double dt, double Mt, double Mc, uwbvec uwb_t, uwbvec uwb_c);
  ~UWBEstimator();

  void Update(Vector<12> visibility, Vector<12> measurement);

  // 伝播のメソッド
  // Ft: ターゲットの制御並進力, Fc: チェイサーの制御並進力
  void Propagate(Vector<3> Ft, Vector<3> Fc);

  Vector<3> GetRelativePosition() const;
  Vector<3> GetRelativeVelocity() const;
  Quaternion GetRelativeAttitude() const;

  void SetQuaternion(Quaternion q_t_i2b, Quaternion q_c_i2b);
  void SetX(Vector<6> x_new);
  bool IsConverged();

  std::string GetLogHeader() const;
  std::string GetLogValue() const;

private:
  double Rinf = 1000000;
  Matrix<12, 6> H;
  Matrix<6, 12> K;
  Matrix<6, 6> M;
  Matrix<6, 6> P, Q, lastP;
  Matrix<12, 12> R;
  Matrix<6, 6> A, B;
  Matrix<6, 6> Gamma, Phi;
  Vector<6> x, Fcontrol_i;
  Vector<12> hx;
  Vector<12> lastObservation;
  // Vector<3> Le, Ve; // 相対位置・速度ベクトルの推定値
  double dt;
  double sigma_Q_tf, sigma_Q_cf;
  double Mc, Mt;
  uwbvec uwb_t, uwb_c;
  Quaternion q_t, q_c; // i2b

  Vector<3> distanceVector(Vector<3> L, Quaternion qt_i2b, Quaternion qc_i2b,
                           Vector<3> post_b, Vector<3> posc_b);

  void SetR(Vector<12> visibility);
  void SetH();

  double CalcDeviation(double distance);
};
