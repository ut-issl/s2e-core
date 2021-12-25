
/*
* @file RWModel.h
* @brief リアクションホイール一個に相当
* @author Kohta Kakihara (refactored by Shun Arahata)
* @date
*/

#ifndef __RWModel_H__
#define __RWModel_H__
#include "../../Library/math/Vector.hpp"
#include "../../Interface/LogOutput/Logger.h"
#include "../../Interface/LogOutput/ILoggable.h"
#include "../Abstract/ComponentBase.h"
#include "rw_ode.hpp"
#include <string>
#include <vector>
#include <limits>

/**
* @brief リアクションホイールのクラス
* @details 一個分に相当する.コンポジションとしてrw_odeを保持.
*/
class RWModel : public ComponentBase, public ILoggable
{
public:
	/**
	* @brief コンストラクタ
	* @param[in]　step_width 積分幅
	* @param[in] init_rpm 初期RPM
	* @param[in] inertia 慣性モーメント
	* @param[in] max_rpm 最高角速度
	* @param[in] motor_drive_init 稼働フラグ
	* @param[in] torque_transition トルク変換行列
	* @param[in] firstorder_lag_const 一次遅れ用定数
	* @param[in] dead_seconds 無駄時間
	* @param[in] target_angular_velocity　目標角速度(=0)
	*/
	RWModel(double step_width,
		double init_rpm,
		double inertia,
		double max_rpm,
		bool motor_drive_init,
		libra::Vector<3> torque_transition,
		double dead_seconds,
		double coasting_end,
		libra::Vector<3> ordinary_lag_coef,
		libra::Vector<3> coasting_lag_coef,
		double max_torque,
		double target_angular_velocity = 0);

	void MainRoutine(int count);
	Vector<3> CalcTorque(double com_period_ms);//!トルク計算、発生
	libra::Vector<3> GetTorque();//!トルク取得(基本的にデバッグ用？)
	bool isMotorDrived();//!動作フラグ取得
	double GetAngularVelocity();//!現在の角速度を取得
	double GetRPM(); //!現在のRPM
	double GetAngularMomentum();//!現在の角運動量を取得(単位はkgm^2/s)
	void SetTorque(double torque = 0, double ctrl_cycle = 0);//!必要なrpm加速度の指定
	void SetLimits(double angular_velocity_upperlimit);	//!最大rpmの設定(単位はrpm)
	void SetDrive(bool);//!動作フラグの設定
	string GetLogHeader() const;
	string GetLogValue() const;
	const double inertia_;//!慣性モーメント edited by ikura
	const double kStepSeconds_;//!積分刻み秒数
	double angular_velocity_limit_;//!角速度絶対値最大値(単位はrad/s)
	const double kCoastingEnd_;

	double GetAngularMomentLimit();

private:
  //void HandlePowerAndCoasting(void);
  RwOde ode_angular_velocity_;//微分方程式のメンバ
  const double kDeadSeconds_;//!無駄時間
  const libra::Vector<3> kTorqueTransition_;//!トルク変換行列(行ベクトルのつもり)
  const libra::Vector<3> kOrdinaryLagCoef_;
  const libra::Vector<3> kCoastingLagCoef_;
  double dead_seconds_;//!残り無駄時間
  double angular_accleration_;
  double angular_velocity_rpm; //角速度をrpmで表示
  double angular_velocity_rad; //角速度をradで表示
  const double MAX_TORQUE_;
  bool motor_drive_;//!動作フラグ(1で動作、0で停止)
  libra::Vector<3> rwtorque_b_;//!トルク保存用、単位はNm
  //libra::Vector<3> angular_momentum_b_;//!RWの角運動量、単位はkgm^2/s(トルク計算には使っていない)
  double angular_momentum_b_;//!RWの角運動量、単位はkgm^2/s(トルク計算には使っていない)
  std::vector<double> targets_angular_accl_;//!無駄時間分のRW各加速度保存
  double target_angular_accl_before_;//!目標回転数1step前情報
};



#endif //__RWModel_H__
