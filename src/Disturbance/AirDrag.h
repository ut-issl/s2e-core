#ifndef __AirDrag_H__
#define __AirDrag_H__

#include <string>
#include <vector>

#include "../Environment/Local/Atmosphere.h"
#include "../Interface/LogOutput/ILoggable.h"
#include "../Library/math/Quaternion.hpp"
#include "../Library/math/Vector.hpp"
#include "SurfaceForce.h"
using libra::Quaternion;
using libra::Vector;

#define K 1.38064852E-23 /* Boltzmann constant */

#pragma once
class AirDrag : public SurfaceForce {
 public:
  AirDrag(const vector<Surface>& surfaces, const Vector<3>& cg_b,
          const double t_w, const double t_m, const double molecular);

  // Override SimpleDisturbance
  virtual void Update(const LocalEnvironment& local_env,
                      const Dynamics& dynamics);

  // Override Loggable
  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;

  // for debug
  void PrintParams(void);
  std::vector<double> cnct;

 private:
  vector<double> Cn_;  // coefficients for out-plane force
  vector<double> Ct_;  // coefficients for in-plane force
  double rho_;         // Air density [kg/m^3]
  double Tw_;          // Temperature of surface [K]
  double Tm_;          // Temperature of atmosphere [K]
  double M_;           // Molecular weight [g/mol]

  // Override SurfaceForce
  void CalcCoef(Vector<3>& vel_b, double air_dens);

  // internal function for calculation
  void CalCnCt(Vector<3>& vel_b);
  double funcPi(double s);
  double funcChi(double s);
};
#endif
