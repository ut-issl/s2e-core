#include <math.h>
#include "../../Library/math/MatVec.hpp"
#include "../../Library/math/ODE.hpp"
#include "RWModule.h"
using namespace libra;

RWModule::RWModule(){
}
RWModule::RWModule(double step_width, 
	double angular_velocity_init_0, double inertia_0, double angular_velocity_upperlimit_init_0, double angular_velocity_lowerlimit_init_0, bool motor_drive_init_0, double angular_accelaration_init_0,
	double angular_velocity_init_1, double inertia_1, double angular_velocity_upperlimit_init_1, double angular_velocity_lowerlimit_init_1, bool motor_drive_init_1, double angular_accelaration_init_1,
	double angular_velocity_init_2, double inertia_2, double angular_velocity_upperlimit_init_2, double angular_velocity_lowerlimit_init_2, bool motor_drive_init_2, double angular_accelaration_init_2,
	double angular_velocity_init_3, double inertia_3, double angular_velocity_upperlimit_init_3, double angular_velocity_lowerlimit_init_3, bool motor_drive_init_3, double angular_accelaration_init_3,
	int number_of_RW, Matrix<3, 4> torque_transition){
	if (number_of_RW == 4){
		RWModel RW0_temp(step_width, angular_velocity_init_0, inertia_0, angular_velocity_upperlimit_init_0, angular_velocity_lowerlimit_init_0, motor_drive_init_0, angular_accelaration_init_0);
		RWModel RW1_temp(step_width, angular_velocity_init_1, inertia_1, angular_velocity_upperlimit_init_1, angular_velocity_lowerlimit_init_1, motor_drive_init_1, angular_accelaration_init_1);
		RWModel RW2_temp(step_width, angular_velocity_init_2, inertia_2, angular_velocity_upperlimit_init_2, angular_velocity_lowerlimit_init_2, motor_drive_init_2, angular_accelaration_init_2);
		RWModel RW3_temp(step_width, angular_velocity_init_3, inertia_3, angular_velocity_upperlimit_init_3, angular_velocity_lowerlimit_init_3, motor_drive_init_3, angular_accelaration_init_3);
		RW_[0] = RW0_temp;
		RW_[1] = RW1_temp;
		RW_[2] = RW2_temp;
		RW_[3] = RW3_temp;
	}else if (number_of_RW == 3){
		RWModel RW0_temp(step_width, angular_velocity_init_0, inertia_0, angular_velocity_upperlimit_init_0, angular_velocity_lowerlimit_init_0, motor_drive_init_0, angular_accelaration_init_0);
		RWModel RW1_temp(step_width, angular_velocity_init_1, inertia_1, angular_velocity_upperlimit_init_1, angular_velocity_lowerlimit_init_1, motor_drive_init_1, angular_accelaration_init_1);
		RWModel RW2_temp(step_width, angular_velocity_init_2, inertia_2, angular_velocity_upperlimit_init_2, angular_velocity_lowerlimit_init_2, motor_drive_init_2, angular_accelaration_init_2);
		RW_[0] = RW0_temp;
		RW_[1] = RW1_temp;
		RW_[2] = RW2_temp;
	}
	torque_transition_ = torque_transition;
}
Vector<3> RWModule::GetTorque(){
	Vector<4> torque_temp;
	for (int i = 0; i < 4; i++){
		torque_temp[i] = RW_[i].GetTorque();
	}
	return torque_transition_ * torque_temp;
}
void RWModule::SetTorque(double angular_accelaration, int module_number){
	RW_[module_number].SetTorque(angular_accelaration);
}
void RWModule::SetLimits(double angular_velocity_upperlimit, double angular_velocity_lowerlimit, int module_number){
	RW_[module_number].SetLimits(angular_velocity_upperlimit, angular_velocity_lowerlimit);
}
void RWModule::SetDrive(bool on, int module_number){
	RW_[module_number].SetDrive(on);
}
string RWModule::WriteLogHeader(void)
{
	string str_tmp = "";

	str_tmp += WriteVector("RWtorque", "b", "Nm", 3);
	for (int i = 0; i < 4; i++){
		str_tmp += RW_[i].WriteLogHeader();
	}

	return str_tmp;
}

string RWModule::WriteLogValue(void)
{
	string str_tmp = "";

	str_tmp += WriteVector(RWtorque_b_);
	for (int i = 0; i < 4; i++){
		str_tmp += RW_[i].WriteLogValue();
	}

	return str_tmp;
}