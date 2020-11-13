#include "STT.h"

#include "../../Library/math/Matrix.hpp"
#include "../../Library/math/GlobalRand.h"
#include "../../Interface/LogOutput/LogUtility.h"

#define _USE_MATH_DEFINES

#include <math.h>
#include <iostream>

using namespace std;
using namespace libra;

STT::STT(ClockGenerator* clock_gen,
  Quaternion& q_b2c,
	double sigma_ortho,
	double sigma_sight,
	double step_time,
	unsigned int output_delay,
	unsigned int output_interval,
	double sun_forbidden_angle,
	double earth_forbidden_angle,
	double moon_forbidden_angle,
	double capture_rate,
	const Dynamics *dynamics,
  const LocalEnvironment* local_env)
	: ComponentBase(1,clock_gen),pos_(0), rot_(g_rand.MakeSeed()), n_ortho_(0.0, sigma_ortho, g_rand.MakeSeed()),
	n_sight_(0.0, sigma_sight, g_rand.MakeSeed()), step_time_(step_time), output_delay_(output_delay),
	output_interval_(output_interval), count_(0), dynamics_(dynamics),local_env_(local_env)
{
	sun_forbidden_angle_ = sun_forbidden_angle;
	earth_forbidden_angle_ = earth_forbidden_angle;
	moon_forbidden_angle_ = moon_forbidden_angle;
	capture_rate_ = capture_rate;

	// 出力Quaternion初期化
	q_stt_i2c_ = Quaternion(0.0, 0.0, 0.0, 1.0);
	q_b2c_ = q_b2c;

	//遅れ生成用配列のサイズを決定，最大遅れの2倍
	MAX_DELAY = int(output_delay_ * 2 / step_time_);
  if(MAX_DELAY <= 0) MAX_DELAY = 1;
	vector<Quaternion> temp(MAX_DELAY);
	q_buffer_ = temp;
	//遅れ生成用配列初期化
	for (int i = 0; i<MAX_DELAY; ++i){ q_buffer_[i] = q_stt_i2c_; }
	
	sight_ = Vector<3>(0);
	ortho1_ = Vector<3>(0);
	ortho2_ = Vector<3>(0);
	sight_[0] = 1;//(1,0,0)@Component coordinates, viewing direction
	ortho1_[1] = 1;//(0,1,0)@Component coordinates, line-of-sight orthogonal direction
	ortho2_[2] = 1;//(0,0,1)@Component coordinates, line-of-sight orthogonal direction

	//デバッグ用初期化，後で消す
	//q_i2b_stt = q_stt_i2c_;
	//エラーフラグ初期化．true : エラーあり， false : なし
	error_flag = true;
}

Quaternion STT::measure(const LocalCelestialInformation *local_celes_info, const Attitude *attinfo)
{
	update(local_celes_info, attinfo); // q_buffer_更新
	if (count_ == 0)
	{
		int hist = pos_ - output_delay_ - 1;
		if (hist < 0){ hist += MAX_DELAY; }
		q_stt_i2c_ = q_buffer_[hist];
		//デバッグ用
		//Quaternion q_c2b = q_b2c_.conjugate();
		//q_i2b_stt = q_stt_i2c_*q_c2b;
	}
	if (++count_ == output_interval_){ count_ = 0; } // intervalごとに値更新
	
	return q_stt_i2c_;//STT観測値
}

libra::Quaternion STT::GetQuaternion() const
{
  return q_stt_i2c_;
}

void STT::update(const LocalCelestialInformation *local_celes_info, const Attitude *attinfo)
{
	Quaternion q_i2b = attinfo->GetQuaternion_i2b();
	//姿勢真値 i2c
	Quaternion  q_stt_temp = q_i2b*q_b2c_;
	// Quaternion representing error rotation around gaze direction
	Quaternion q_sight(sight_, n_sight_);
	// 視線垂直面の回転軸方向(範囲[0:2pi]の一様乱数)
	double rot = 2.0*M_PI*double(rot_);
	// 視線直交面内誤差回転軸の計算
	Vector<3> rot_axis = cos(rot)*ortho1_ + sin(rot)*ortho2_;
	// 回転軸回りの誤差回転を表すQuaternion
	Quaternion q_ortho(rot_axis, n_ortho_);
	AllJudgement(local_celes_info, attinfo);

	// 慣性座標→STT座標→視線方向回転→視線直交方向回転
	q_buffer_[pos_] = q_stt_temp*q_sight*q_ortho;
	// q_buffer_の記録位置更新
	++pos_; pos_ %= MAX_DELAY;


}

void STT::AllJudgement(const LocalCelestialInformation *local_celes_info, const Attitude *attinfo)
{
	int judgement = 0;
	judgement = SunJudgement(local_celes_info->GetPosFromSC_b("SUN"));
	judgement+= EarthJudgement(local_celes_info->GetPosFromSC_b("EARTH"));
	judgement+= MoonJudgement(local_celes_info->GetPosFromSC_b("MOON"));
	judgement+= CaptureRateJudgement(attinfo->GetOmega_b());
	if (judgement > 0){
		error_flag = true;
	}
	else
		error_flag = false;
}

int STT::SunJudgement(const libra::Vector<3>& sun_b)//太陽禁止角判定,半頂角が禁止角である円錐の底面に太陽が入っているかどうかで禁止角を判定する
{
	Quaternion q_c2b = q_b2c_.conjugate();
	Vector<3> sight_b = q_c2b.frame_conv(sight_);
	double sun_angle_rad = CalAngleVect_rad(sun_b, sight_b);
	if (sun_angle_rad < sun_forbidden_angle_){
		return 1;
	}
	else
		return 0;
}

int STT::EarthJudgement(const libra::Vector<3>& earth_b)//地球禁止角判定
{
	Quaternion q_c2b = q_b2c_.conjugate();
	Vector<3> sight_b = q_c2b.frame_conv(sight_);
	double earth_angle_rad = CalAngleVect_rad(earth_b, sight_b);
	if (earth_angle_rad < earth_forbidden_angle_)
	{
		return 1;
	}
	else
		return 0;
}

int STT::MoonJudgement(const libra::Vector<3>& moon_b)//月禁止角判定
{
	Quaternion q_c2b = q_b2c_.conjugate();
	Vector<3> sight_b = q_c2b.frame_conv(sight_);
	double moon_angle_rad = CalAngleVect_rad(moon_b, sight_b);
	if (moon_angle_rad < moon_forbidden_angle_)
	{
		return 1;
	}
	else
		return 0;
}

int STT::CaptureRateJudgement(const libra::Vector<3>& omega_b)//捕捉レート判定
{
	double omega_norm = norm(omega_b);
	if (omega_norm > capture_rate_)
	{
		return 1;
	}
	else
		return 0;
}

string STT::GetLogHeader() const
{
	string str_tmp = "";

	str_tmp += WriteVector("quaternion_STT", "i2c", "-", 4);
	str_tmp += WriteScalar("STT error flag");

	return str_tmp;
}

string STT::GetLogValue() const
{
	string str_tmp = "";

	str_tmp += WriteQuaternion(q_stt_i2c_);
	str_tmp += WriteScalar(double(error_flag));

	return str_tmp;
}

double STT::CalAngleVect_rad(const Vector<3>& vect1, const Vector<3>& vect2)
{
	Vector<3> vect1_normal(vect1); normalize(vect1_normal); //Vector1 正規化
	Vector<3> vect2_normal(vect2); normalize(vect2_normal); //Vector2 正規化
	double cosTheta = inner_product(vect1_normal, vect2_normal); //cos計算
	double theta_rad = acos(cosTheta);
	return theta_rad;
}

void STT::MainRoutine(int count)
{
    ReceiveCommand();
    measure(&(local_env_->GetCelesInfo()), &(dynamics_->GetAttitude()));
    SendTelemetry();
}

void STT::ReceiveCommand() {}
void STT::SendTelemetry() {}
