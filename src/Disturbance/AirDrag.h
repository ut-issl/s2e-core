#ifndef __AirDrag_H__
#define __AirDrag_H__

#include "../Library/math/Vector.hpp"
#include "../Library/math/Quaternion.hpp"
#include "SurfaceForce.h"
#include "../Environment/Atmosphere.h"
#include "../Interface/LogOutput/ILoggable.h"
using libra::Vector;
using libra::Quaternion;

#define K     1.38064852E-23   /* Boltzmann constant */

#pragma once
class AirDrag : public SurfaceForce
{
public:
  AirDrag(const Vector<3>& px_arm,
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
    const Vector<3>& center,
    const Vector<6>& specularity,
    const double t_w,
    const double t_m,
    const double molecular);

  // Override SimpleDisturbance
  virtual void Update(Envir & env, const Spacecraft & spacecraft);

  // Override Loggable
  virtual string GetLogHeader() const;
  virtual string GetLogValue() const;

  //for debug
  void PrintParams(void);
  double cnct[6];

private:
  double Cn[6]; //coefficients for out-plane force
  double Ct[6]; //coefficients for in-plane force
  double rho;   //Air density [kg/m^3]
  double Tw_;   //Temperature of surface [K]
  double Tm_;   //Temperature of atmosphere [K]
  double M;     //Molecular weight [g/mol]

  // Override SurfaceForce
  void CalcCoef(Vector<3>& vel_b, double air_dens);

  // internal function for calculation
  void CalCnCt(Vector<3>& vel_b);
  double funcPi(double s);
  double funcChi(double s);
};
#endif
