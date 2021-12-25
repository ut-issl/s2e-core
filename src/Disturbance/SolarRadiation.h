//
//  SolarRadiation.h
//
//
//  Created by KakiharaKota on 2016/02/26.
//
//

#ifndef __SolarRadiation_h__
#define __SolarRadiation_h__
#include "../Interface/LogOutput/ILoggable.h"
#include "SurfaceForce.h"
#include "../Library/math/Vector.hpp"
using libra::Vector;

class SolarRadiation : public SurfaceForce
{
public:
  SolarRadiation(const Vector<3>& px_arm,
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
    const Vector<6>& reflectivity,
    const Vector<6>& specularity
  );

  // Override SimpleDisturbance
  virtual void Update(Envir& env, const Spacecraft & spacecraft);

  // Override Loggable
  virtual string GetLogHeader() const;
  virtual string GetLogValue() const;

private:
  // Override SurfaceForce
  virtual void CalcCoef(Vector<3>& input_b, double item);
};

#endif /* SolarRadiation_h */
