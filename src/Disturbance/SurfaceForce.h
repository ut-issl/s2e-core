#pragma once

#ifndef __SurfaceForce_H__
#define __SurfaceForce_H__

#include "SimpleDisturbance.h"
#include "../Library/math/Vector.hpp"
#include "../Library/math/Quaternion.hpp"
using libra::Vector;
using libra::Quaternion;

class SurfaceForce : public SimpleDisturbance
{
public:
  SurfaceForce(const Vector<3>& px_arm,
    const Vector<3>& mx_arm,
    const Vector<3>& py_arm,
    const Vector<3>& my_arm,
    const Vector<3>& pz_arm,
    const Vector<3>& mz_arm,
    const Vector<6>& areas,
    const Vector<3>& px_normal,
    const Vector<3>& mx_normal,
    const Vector<3>& py_normal,
    const Vector<3>& my_normal,
    const Vector<3>& pz_normal,
    const Vector<3>& mz_normal,
    const Vector<3>& center);
  virtual ~SurfaceForce() { }
  
protected:
  // Spacecraft Structure parameters
  double area_[6];//Area of surfaces
  Vector<3> NormalVect_b[6]; //Normal vector of the surfaces @ Body Frame
  Vector<3> arms_b[6];//Position vector of the surfaces @ Body Frame
  Vector<3> cent_b;//Position vector of the center of mass @ Body frame

  // Optical properties
  double reflectivity_[6];  //Total reflectance
  double specularity_[6];   //Specularity

  // Internal calculated variables
  double normal_coef_[6];//coefficients for out-plane force
  double tangential_coef_[6];//coefficients for in-plane force
  double cosX[6];
  double sinX[6];

  // Functions
  Vector<3> CalcTorqueForce(Vector<3>& input_b, double item);
  void CalcTheta(Vector<3>& input_b);

  // Functions to be defined by sub classes
  virtual void CalcCoef(Vector<3>& input_b, double item) = 0;
};
#endif
