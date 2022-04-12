#pragma once
#include <Environment/Global/CelestialInformation.h>

#include "Orbit.h"

static elsetrec satrec;

class EarthCenteredOrbit : public Orbit {
 public:
  EarthCenteredOrbit(const CelestialInformation* celes_info, char* tle1, char* tle2, int wgs,
                     double current_jd);  //地球周回軌道用コンストラクタ

  virtual void Propagate(double endtime,
                         double current_jd);  //軌道のプロパゲーション

  Vector<3> GetESIOmega();

  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;

 private:
  gravconsttype whichconst_;
};
