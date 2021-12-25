#pragma once
#include "../../Library/math/Vector.hpp"
using libra::Vector;

template<size_t N>
class PIDController
{
public:
  PIDController(double dt);
  ~PIDController();

  // PID計算をして出力（制御トルクなど）を計算
  Vector<N> CalcOutput(Vector<N> p_error);

  // PID計算をして出力（制御トルクなど）を計算
  Vector<N> CalcOutput(Vector<N> p_error, Vector<N> current_d);

  // ゲインを設定
  void SetGains(double p, double d, double i);

private:
  bool isFirst = true;
  double pGain = 0;
  double dGain = 0;
  double iGain = 0;
  double dt = 1;
  
  // 1ステップ前の状態量
  Vector<N> last_p;

  // 積分項
  Vector<N> term_i;
};


template<size_t N>
PIDController<N>::PIDController(double dt_)
{
  dt = dt_;
  last_p = Vector<N>(0);
  term_i = Vector<N>(0);
}

template<size_t N>
PIDController<N>::~PIDController()
{
}

template<size_t N>
Vector<N> PIDController<N>::CalcOutput(Vector<N> p_error)
{
  if (isFirst)
  {
    isFirst = false;
    last_p = p_error;
  }
  Vector<N> dvec = (p_error - last_p) ;
  dvec /= dt;
  last_p = p_error;

  return CalcOutput(p_error, dvec);
}

template<size_t N>
inline Vector<N> PIDController<N>::CalcOutput(Vector<N> p_error, Vector<N> current_d)
{
  auto dvec = current_d;
  dvec *= 1;

  term_i += dt * p_error;
  Vector<N> ivec = term_i;

  Vector<N> pvec = p_error;

  Vector<N> output = pGain * pvec + dGain * dvec + iGain * ivec;
  return output;
}

template<size_t N>
void PIDController<N>::SetGains(double p, double d, double i)
{
  pGain = p;
  dGain = d;
  iGain = i;
}
