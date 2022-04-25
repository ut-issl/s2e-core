//
//  SolarRadiation.h
//
//
//  Created by KakiharaKota on 2016/02/26.
//
//

#ifndef __SolarRadiation_h__
#define __SolarRadiation_h__
#include <Library/utils/Macros.hpp>
#include <string>

#include "../Interface/LogOutput/ILoggable.h"
#include "../Library/math/Vector.hpp"
#include "SurfaceForce.h"
using libra::Vector;

class SolarRadiation : public SurfaceForce {
 public:
  SolarRadiation(const vector<Surface>& surfaces, const Vector<3>& cg_b);

  // Override SimpleDisturbance
  virtual void Update(const LocalEnvironment& local_env, const Dynamics& dynamics);

  // Override Loggable
  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;

 private:
  // Override SurfaceForce
  virtual void CalcCoef(Vector<3>& input_b, double item);
};

#endif /* SolarRadiation_h */
