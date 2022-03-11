#include "LPF.h"

#include <cmath>

LPF::LPF(double omega_c, double sampling_t)
{
	using std::pow;

	double gamma = 0.5*omega_c*sampling_t;
	double gamma_2 = pow(gamma, 2.0);
	double coef = 1.0 / pow(1 + gamma, 2.0);

	// フィルタ係数初期化
	a_[0] = 0.0; // 使用しない。
	a_[1] = coef*2.0*(gamma_2 - 1.0);
	a_[2] = coef*pow(1.0 - gamma, 2.0);

	b_[0] = coef*gamma_2;
	b_[1] = 2.0*b_[0];
	b_[2] = b_[0];


	// 入出力記憶の初期化(ここではすべて0.0とする)
	for (int i = 0; i<3; ++i) { y_[i] = u_[i] = 0.0; }
}


double LPF::operator()(double u)
{
	// 入力バッファシフト
	u_[2] = u_[1]; u_[1] = u_[0]; u_[0] = u;
	// 出力バッファシフト
	y_[2] = y_[1]; y_[1] = y_[0];

	y_[0] = b_[0] * u_[0];
	for (int i = 1; i<3; ++i){ y_[0] += -a_[i] * y_[i] + b_[i] * u_[i]; }

	return y_[0];
}
