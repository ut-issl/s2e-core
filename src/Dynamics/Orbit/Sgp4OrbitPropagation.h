#pragma once
#include <Environment/Global/CelestialInformation.h>
#include <Library/sgp4/sgp4io.h>
#include <Library/sgp4/sgp4unit.h>

#include "Orbit.h"

class Sgp4OrbitPropagation : public Orbit {
 public:
  Sgp4OrbitPropagation(const CelestialInformation* celes_info, char* tle1, char* tle2, int wgs, double current_jd);

  virtual void Propagate(double endtime, double current_jd);

  Vector<3> GetESIOmega();

  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;

 private:
  gravconsttype whichconst_;
  elsetrec satrec_;
  const CelestialInformation* celes_info_;
};
