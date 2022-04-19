#include "UWBEstimator.h"

#include "../AOCS/UWBSensor.h"

UWBEstimator::UWBEstimator(double dt, double Mt, double Mc, uwbvec uwb_t, uwbvec uwb_c) {
  this->sigma_Q_tf = 0.001;
  this->sigma_Q_cf = 0.001;
  this->dt = dt;
  this->Mc = Mc;
  this->Mt = Mt;
  this->uwb_t = uwb_t;
  this->uwb_c = uwb_c;

  fill_up(P, 0.0);
  fill_up(Q, 0.0);
  fill_up(R, 0.0);
  fill_up(H, 0.0);
  fill_up(K, 0.0);
  fill_up(M, 0.0);
  fill_up(A, 0.0);
  fill_up(B, 0.0);
  fill_up(lastP, 0.0);
  // Matrix<6, 6> A(0), B(0);
  for (auto i = 0; i < 3; i++) {
    A(i, i + 3) = 1;
    B(i + 3, i) = 1 / Mt;
    B(i + 3, i + 3) = -1 / Mc;
    P(i, i) = 100;
    P(i + 3, i + 3) = 3;
    Q(i, i) = sigma_Q_tf * sigma_Q_tf;
    Q(i + 3, i + 3) = sigma_Q_cf * sigma_Q_cf;
  }
  SetR(Vector<12>(1));
  Phi = libra::eye<6>() + dt * A;
  Gamma = dt * B;
  x *= 0;
  hx *= 0;
  Fcontrol_i *= 0;
  lastObservation *= 0;
}

UWBEstimator::~UWBEstimator() {}

void UWBEstimator::Update(Vector<12> visibility, Vector<12> measurement) {
  lastObservation = measurement;
  lastP = P;
  SetR(visibility);
  SetH();
  M = Phi * P * transpose(Phi) + Gamma * Q * transpose(Gamma);
  Matrix<12, 12> inv;
  try {
    inv = invert(H * M * transpose(H) + R);
  } catch (...) {
    fill_up(inv, 0.0);
    std::cout << "singular!!\r\n";
  }

  K = M * transpose(H) * inv;
  auto t3 = measurement - hx;
  auto t4 = K * (measurement - hx);
  x = x + K * (measurement - hx);
  // Joseph Form https://wolfweb.unr.edu/~fadali/EE782/DiscreteKF.pdf
  auto temp = libra::eye<6>() - K * H;
  P = temp * M * transpose(temp) + K * R * transpose(K);
}

void UWBEstimator::Propagate(Vector<3> Ft, Vector<3> Fc) {
  lastP = P;
  for (auto i = 0; i < 3; i++) {
    Fcontrol_i[i] = Ft[i];
    Fcontrol_i[i + 3] = Fc[i];
  }
  x = Phi * x + Gamma * Fcontrol_i;
  P = Phi * P * transpose(Phi) + Gamma * Q * transpose(Gamma);
}

Vector<3> UWBEstimator::GetRelativePosition() const {
  Vector<3> Le;
  for (auto i = 0; i < 3; i++) Le[i] = x[i];
  return Le;
}

Vector<3> UWBEstimator::GetRelativeVelocity() const {
  Vector<3> Le_dot;
  for (auto i = 0; i < 3; i++) Le_dot[i] = x[i + 3];
  return Le_dot;
}

void UWBEstimator::SetQuaternion(Quaternion q_t_i2b, Quaternion q_c_i2b) {
  q_t = q_t_i2b;
  q_c = q_c_i2b;
}

void UWBEstimator::SetX(Vector<6> x_new) { x = x_new; }

bool UWBEstimator::IsConverged() {
  auto Pdiff = P - lastP;
  bool isconverged = true;
  for (size_t i = 0; i < Pdiff.row(); i++) {
    // 最初は1mで捕捉できてれば十分
    isconverged &= std::abs(Pdiff[i][i]) < 1e-2;
  }
  return isconverged;
}

std::string UWBEstimator::GetLogHeader() const {
  std::string str_tmp = "";
  str_tmp += WriteVector("RelPosEst", "i", "m", 3);
  str_tmp += WriteVector("RelVelEst", "i", "m/s", 3);
  str_tmp += WriteVector("UWBObs", "i", "m", 12);
  str_tmp += WriteVector("Fcontrol", "i", "N", 6);
  return str_tmp;
}

std::string UWBEstimator::GetLogValue() const {
  std::string str_tmp = "";
  str_tmp += WriteVector(x);
  str_tmp += WriteVector(lastObservation);
  str_tmp += WriteVector(Fcontrol_i);
  return str_tmp;
}

Vector<3> UWBEstimator::distanceVector(Vector<3> L, Quaternion qt_i2b, Quaternion qc_i2b, Vector<3> post_b, Vector<3> posc_b) {
  return L + qt_i2b.frame_conv_inv(post_b) - qc_i2b.frame_conv_inv(posc_b);
}

void UWBEstimator::SetR(Vector<12> visibility) {
  auto dist_e = norm(GetRelativePosition());
  double sigma_R = CalcDeviation(dist_e);
  for (auto i = 0; i < 12; i++) {
    R[i][i] = visibility[i] ? sigma_R * sigma_R : Rinf;
  }
}

void UWBEstimator::SetH() {
  for (auto i = 0; i < 3; i++) {
    auto uwbc = uwb_c[i];
    for (auto j = 0; j < 4; j++) {
      auto uwbt = uwb_t[j];
      auto Le = GetRelativePosition();
      auto relpos = distanceVector(Le, q_t, q_c, uwbt->GetPos_b(), uwbc->GetPos_b());
      auto relposnorm = norm(relpos);
      hx[i * 4 + j] = relposnorm;
      auto diffs = 1 / relposnorm * relpos;
      for (auto k = 0; k < 3; k++) {
        H[i * 4 + j][k] = diffs[k];
      }
    }
  }
}

double UWBEstimator::CalcDeviation(double distance) {
  double uwb_error = 0.1;
  if (distance < 200 && distance >= 150)
    uwb_error = 0.07;
  else if (distance < 150 && distance >= 100)
    uwb_error = 0.05;
  else if (distance < 100)
    uwb_error = 0.03;
  else
    uwb_error = 0.1;

  // STT誤差を考慮すると、観測ノイズの偏差は増加させる必要あり
  // 200mで100が良かった 比例と仮定
  // いまいち根拠薄いが😩
  double stt_error_fac = 100 * (distance) / 200.0 + 1;

  return uwb_error * stt_error_fac;
}
