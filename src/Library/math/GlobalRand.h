#ifndef GLOBALRAND_HPP_
#define GLOBALRAND_HPP_
#include "./Ran0.hpp"

class GlobalRand
{
public:
  //! コンストラクタ
  /*!
    \param q_bd 機体座標(B)からSTT設計座標(D)への変換Quaternion
    \param q_ds STT設計座標(D)からSTT実座標(S)への変換Quaternion
    \param sigma_ortho 視線直交方向誤差標準偏差[rad/sec]
    \param sigma_sight 視線方向誤差標準偏差[rad/sec]
    \param delay STT出力遅れ。0.1秒単位範囲[0-MAX_DELAY]。
  */
	GlobalRand();
    void SetSeed(long seed);
	long MakeSeed();

private:
	static const unsigned int MAX_SEED = 0xffffffff;
	libra::Ran0 base_rand_;
	long seed_;
};

extern GlobalRand g_rand;


#endif