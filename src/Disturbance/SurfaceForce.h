#pragma once

#ifndef __SurfaceForce_H__
#define __SurfaceForce_H__

#include "SimpleDisturbance.h"
#include "Surface.h"
#include "../Library/math/Vector.hpp"
#include "../Library/math/Quaternion.hpp"
using libra::Vector;
using libra::Quaternion;

#include <vector>

class SurfaceForce : public SimpleDisturbance
{
public:
  SurfaceForce(const vector<Surface>& surfaces, const Vector<3>& cg_b);
  virtual ~SurfaceForce() { }
  
protected:
  // Spacecraft Structure parameters
  const vector<Surface>& surfaces_;
  Vector<3> cg_b_;//Position vector of the center of mass @ Body frame

  // Internal calculated variables
  vector<double> normal_coef_;     //coefficients for out-plane force
  vector<double> tangential_coef_; //coefficients for in-plane force
  vector<double> cosX;
  vector<double> sinX;

  // Functions
  Vector<3> CalcTorqueForce(Vector<3>& input_b, double item);
  void CalcTheta(Vector<3>& input_b);

  // Functions to be defined by sub classes
  virtual void CalcCoef(Vector<3>& input_b, double item) = 0;
};
#endif
