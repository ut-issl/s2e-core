#include <math.h>
#include <Library/math/MatVec.hpp>
#include <Interface/LogOutput/Logger.h>
#include <Library/math/ODE.hpp>
#include "RWModel.h"
using namespace libra;

class RWModule{
private:
	RWModel RW_[4];
	Matrix<3, 4> torque_transition_;//トルク変換行列
	Vector<3> RWtorque_b_;//出したトルク(ログ用)
public:
	RWModule();
	RWModule(double step_width,
		double angular_velocity_init_0, double inertia_0, double angular_velocity_upperlimit_init_0, double angular_velocity_lowerlimit_init_0, bool motor_drive_init_0, double angular_accelaration_init_0,
		double angular_velocity_init_1, double inertia_1, double angular_velocity_upperlimit_init_1, double angular_velocity_lowerlimit_init_1, bool motor_drive_init_1, double angular_accelaration_init_1,
		double angular_velocity_init_2, double inertia_2, double angular_velocity_upperlimit_init_2, double angular_velocity_lowerlimit_init_2, bool motor_drive_init_2, double angular_accelaration_init_2,
		double angular_velocity_init_3, double inertia_3, double angular_velocity_upperlimit_init_3, double angular_velocity_lowerlimit_init_3, bool motor_drive_init_3, double angular_accelaration_init_3,
		int number_of_RW, Matrix<3, 4> torque_transition);
	Vector<3> GetTorque();//トルク発生
	void SetTorque(double angular_accelaration, int module_number);//必要な角加速度の指定(単位はrpm/s)
	void SetLimits(double angular_velocity_upperlimit, double angular_velocity_lowerlimit, int module_number);//UL/LLの設定(単位はrpm)
	void SetDrive(bool on, int module_number);//動作フラグの設定
	string WriteLogHeader(void);//ログofヘッダー
	string WriteLogValue(void);//ログof値
};
