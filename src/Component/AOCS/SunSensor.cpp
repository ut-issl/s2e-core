#include "SunSensor.h"
#include "../../Library/math/NormalRand.hpp"
using libra::NormalRand;
#include "../../Library/math/GlobalRand.h"
#include "../../Interface/LogOutput/LogUtility.h"

#define _USE_MATH_DEFINES
#include <math.h>

using namespace std;

SunSensor::SunSensor(const libra::Quaternion& q_b2c,
					 double detectable_angle_rad,
				     double ss_wnvar,
					 double ss_bivar,
                     const Dynamics* dynamics)
	:detectable_angle_rad_(detectable_angle_rad),
	 ss_wnvar_(ss_wnvar),
	 ss_bivar_(ss_bivar),
	 dynamics_(dynamics)
{
	q_b2c_ = q_b2c;
	sun_c_ = Vector<3>(0);
	measured_sun_c_ = Vector<3>(0);
	sun_detected_flag_ = false;

	static NormalRand nr(0.0, ss_bivar_, g_rand.MakeSeed());
	ss_bias_ = nr;
}

void SunSensor::MainRoutine(int time_count)
{
    measure(dynamics_->GetCelestial().GetPosFromSC_b("SUN"), false);
}

void SunSensor::measure(const Vector<3>& sun_b, bool sun_eclipsed)
{
	sun_c_ = q_b2c_.frame_conv(sun_b); //太陽方向ベクトルを機体座標系からセンサ座標系へ変換

	SunDetectionJudgement(sun_eclipsed); //太陽がセンサ視野内あるかの判断

	//太陽がセンサ視野内にある場合
	if (sun_detected_flag_)
	{
		alpha_ = atan2(sun_c_[0], sun_c_[2]);
		beta_ = atan2(sun_c_[1], sun_c_[2]);

		static NormalRand nr1(ss_bias_, ss_wnvar_, g_rand.MakeSeed());
		static NormalRand nr2(ss_bias_, ss_wnvar_, g_rand.MakeSeed());
		alpha_ += nr1;
		beta_ += nr2;

		//-π/2〜π/2にする
		SetTanRange(alpha_);
		SetTanRange(beta_);

		measured_sun_c_[0] = tan(alpha_);
		measured_sun_c_[1] = tan(beta_);
		measured_sun_c_[2] = 1.0;

		normalize(measured_sun_c_); //正規化
	}
	//太陽がセンサ視野内にない場合
	else
	{
		measured_sun_c_ = Vector<3>(0);
		alpha_ = 0.0;
		beta_ = 0.0;
	}
}

void SunSensor::SunDetectionJudgement(bool sun_eclipsed)
{
	Vector<3> sun_direction_c = normalize(sun_c_);

	sun_angle_ = acos(sun_direction_c[2]);

	//視野角内に太陽あるかの判定
	if (sun_eclipsed)
	{
		sun_detected_flag_ = false;
	}
	else{
		if (sun_angle_ < detectable_angle_rad_)
		{
			sun_detected_flag_ = true;
		}
		else{
			sun_detected_flag_ = false;
		}
	}
}

bool SunSensor::GetSunDetected()
{
	return sun_detected_flag_;
}

Vector<3> SunSensor::GetMeasuredSun_c()
{
	return measured_sun_c_;
}

Vector<3> SunSensor::GetMeasuredSun_b()
{
	return  q_b2c_.conjugate().frame_conv(measured_sun_c_); //太陽方向ベクトルをセンサ座標系から機体座標系へ変換
}
//太陽角(α)の取得用関数
double SunSensor::GetSunAngleAlpha()
{
	return alpha_;
}

//太陽角(β)の取得用関数
double SunSensor::GetSunAngleBeta()
{
	return beta_;
}

void SunSensor::SetTanRange(double x)
{
	if (x>  M_PI / 2) x = M_PI - x;
	if (x< -M_PI / 2) x = -M_PI - x;
}

string SunSensor::GetLogHeader() const
{
	string str_tmp = "";

	str_tmp += WriteVector("sun", "c", "-", 3);
	str_tmp += WriteScalar("sun_detected_flag","-");;

	return str_tmp;
}

string SunSensor::GetLogValue() const
{
	string str_tmp = "";

	str_tmp += WriteVector(measured_sun_c_);
	str_tmp += WriteScalar(double(sun_detected_flag_));

	return str_tmp;
}
