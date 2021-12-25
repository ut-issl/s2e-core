#pragma once

#ifndef __STT_H__
#define __STT_H__

#include "../../Library/math/Quaternion.hpp"
#include "../../Library/math/Vector.hpp"
#include "../../Library/math/Ran1.hpp"
#include "../../Library/math/NormalRand.hpp"
#include "../../Interface/LogOutput/ILoggable.h"
#include "../../Environment/CelestialInformation.h"
#include "../../Dynamics/Attitude/Attitude.h"
#include "../Abstract/ComponentBase.h"
#include "../../Dynamics/Dynamics.h"
#include<vector>

class STT: public ComponentBase, public ILoggable
{
public:
	//! コンストラクタ
	/*!
	\param q_b2c 機体座標(B)からコンポーネント座標(C)への変換Quaternion
	\param sigma_ortho 視線直交方向誤差標準偏差[rad/sec]
	\param sigma_sight 視線方向誤差標準偏差[rad/sec]
	\param step_time STT計算間隔
	\param output_delay STT出力遅れ。単位:step_time,[0-MAX_DELAY]。
	\param output_interval STT出力間隔。単位:step_time
	\param forbidden_angle 太陽禁止角[rad]．
	\param capture_rate 捕捉可能レート[rad/s]
	*/
	STT(libra::Quaternion& q_b2c,
		double sigma_ortho,
		double sigma_sight,
		double step_time,
		unsigned int output_delay,
		unsigned int output_interval,
		double sun_forbidden_angle,
		double earth_forbidden_angle,
		double moon_forbidden_angle,
		double capture_rate,
		const Dynamics *dynamics);

	void  MainRoutine(int count) override;

	libra::Quaternion measure(CelestialInformation *celestial_info, Attitude *attinfo);//姿勢取得
    libra::Quaternion GetQuaternion() const;
	int SunJudgement(const libra::Vector<3>& sun_b);//太陽禁止角判定
	int EarthJudgement(const libra::Vector<3>& earth_b);//地球禁止角判定
	int MoonJudgement(const libra::Vector<3>& moon_b);//月禁止角判定
	int CaptureRateJudgement(const libra::Vector<3>& omega_b);//捕捉レート判定
	void update(CelestialInformation *celestial_info, Attitude *attinfo);
	virtual string GetLogHeader() const;
	virtual string GetLogValue() const;
	//! 異常検知bool true:異常あり，　false:異常なし
	bool error_flag;

private:
	int MAX_DELAY;

	//! STT出力値,
	libra::Quaternion q_stt_i2c_;
	//! STTの姿勢b2c
	libra::Quaternion q_b2c_;
	//! STT出力バッファ。時間遅れ生成用。
	std::vector<Quaternion> q_buffer_;
	//! バッファ位置カウンタ
	int pos_;
	//太陽禁止角度[rad]
	double sun_forbidden_angle_;
	//地球禁止角度[rad]
	double earth_forbidden_angle_;
	//月禁止角度[rad]
	double moon_forbidden_angle_;
	//捕捉可能角速度[rad/s]
	double capture_rate_;
	//! 視線方向ベクトル(STT座標系表記)
	libra::Vector<3> sight_;
	//! 視線直交方向ベクトル1(STT座標系表記)
	libra::Vector<3> ortho1_;
	//! 視線直交方向ベクトル2(STT座標系表記)
	libra::Vector<3> ortho2_;
	//! 視線直交方向面内の誤差回転軸方向を決める乱数
	libra::Ran1 rot_;
	//! STT視線直交方向誤差[rad]
	libra::NormalRand n_ortho_;
	//! STT視線方向誤差[rad]
	libra::NormalRand n_sight_;
	//! STT計算時間間隔．シミュレーションの更新間隔と同じ
	double step_time_;
	//! STT出力遅れ。step_time単位
	unsigned int output_delay_;
	//! STT出力間隔。step_time単位
	unsigned int output_interval_;
	//! STT出力更新カウンタ
	std::size_t count_;

    const Dynamics* dynamics_;

	//2つのベクトルの角度を計算する関数
	double CalAngleVect_rad(const libra::Vector<3>& vect1, const libra::Vector<3>& vect2);
	//Judgement系関数総括
	void AllJudgement(CelestialInformation *celestial_info, Attitude *attinfo);
	
	//デバッグ用
	//libra::Quaternion q_i2b_stt;

	void ReceiveCommand();
	void SendTelemetry();
};


#endif