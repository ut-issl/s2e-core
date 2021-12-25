#include "SurfaceForce.h"
#include "../Library/math/Vector.hpp"
using libra::Vector;
using libra::Quaternion;

using namespace libra;

SurfaceForce::SurfaceForce(const Vector<3>& px_arm,
	                       const Vector<3>& mx_arm,
						   const Vector<3>& py_arm,
						   const Vector<3>& my_arm,
						   const Vector<3>& pz_arm,
						   const Vector<3>& mz_arm,
						   const Vector<6>& area,
						   const Vector<3>& px_normal,
	                       const Vector<3>& mx_normal,
						   const Vector<3>& py_normal,
						   const Vector<3>& my_normal,
						   const Vector<3>& pz_normal,
						   const Vector<3>& mz_normal,
						   const Vector<3>& center)
{
	for (int i=0; i<6; ++i)
	{
		area_[i]=area[i];
	}
	arms_b[0][0] = px_arm[0] ; arms_b[0][1] = px_arm[1]  ; arms_b[0][2] = px_arm[2];
	arms_b[1][0] = mx_arm[0] ; arms_b[1][1] = mx_arm[1]  ; arms_b[1][2] = mx_arm[2];
	arms_b[2][0] = py_arm[0] ; arms_b[2][1] = py_arm[1]  ; arms_b[2][2] = py_arm[2];
	arms_b[3][0] = my_arm[0] ; arms_b[3][1] = my_arm[1]  ; arms_b[3][2] = my_arm[2];
	arms_b[4][0] = pz_arm[0] ; arms_b[4][1] = pz_arm[1]  ; arms_b[4][2] = pz_arm[2];
	arms_b[5][0] = mz_arm[0] ; arms_b[5][1] = mz_arm[1]  ; arms_b[5][2] = mz_arm[2];

	NormalVect_b[0][0] = px_normal[0] ; NormalVect_b[0][1] = px_normal[1] ; NormalVect_b[0][2] = px_normal[2];
	NormalVect_b[1][0] = mx_normal[0] ; NormalVect_b[1][1] = mx_normal[1] ; NormalVect_b[1][2] = mx_normal[2];
	NormalVect_b[2][0] = py_normal[0] ; NormalVect_b[2][1] = py_normal[1] ; NormalVect_b[2][2] = py_normal[2];
	NormalVect_b[3][0] = my_normal[0] ; NormalVect_b[3][1] = my_normal[1] ; NormalVect_b[3][2] = my_normal[2];
	NormalVect_b[4][0] = pz_normal[0] ; NormalVect_b[4][1] = pz_normal[1] ; NormalVect_b[4][2] = pz_normal[2];
	NormalVect_b[5][0] = mz_normal[0] ; NormalVect_b[5][1] = mz_normal[1] ; NormalVect_b[5][2] = mz_normal[2];

	cent_b[0]= center[0] ; cent_b[1]= center[1] ; cent_b[2]= center[2];

	force_b_ = Vector<3>(0);
	torque_b_ = Vector<3>(0);
}

// input_b: direction of disturbance source @ body frame
// item: parameter which decide the magnitude of the disturbances (Solar flux, air density)
Vector<3> SurfaceForce::CalcTorqueForce(Vector<3>& input_b, double item)
{
	CalcTheta(input_b);
	CalcCoef(input_b, item);
	Vector<3> Force(0.0);
	Vector<3> Trq(0.0);
	Vector<3> input_b_normal(input_b); 
  normalize(input_b_normal);

	for (int i = 0; i<6 ; i++){
		if (cosX[i]>0){ // if the surface directs to the disturbance source (sun or air)
      // calc direction of in-plane force
			Vector<3> ncu = outer_product(input_b_normal,NormalVect_b[i]);
			Vector<3> ncu_normalized = normalize(ncu);
			Vector<3> s = outer_product(ncu_normalized,NormalVect_b[i]);
      // calc force
			Vector<3> Fs = -1.0 * normal_coef_[i] * NormalVect_b[i] + tangential_coef_[i] * s;
			Force += Fs;
      // calc torque 
			Vector<3> Ts = outer_product(arms_b[i]-cent_b,Fs);
			Trq += Ts;
		}
	}
	force_b_ = Force;
	torque_b_ = Trq;
	return torque_b_;
}

// input_b: direction of disturbance source @ body frame
void SurfaceForce::CalcTheta(Vector<3>& input_b)
{
	Vector<3> input_b_normal(input_b); 
  normalize(input_b_normal);

	for (int i = 0 ; i < 6 ; ++i){
		cosX[i] = inner_product(NormalVect_b[i], input_b_normal);
		sinX[i] = sqrt(1-cosX[i]*cosX[i]);
	}
}
