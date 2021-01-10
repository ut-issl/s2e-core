#pragma once
#include "Orbit.h"

static elsetrec satrec;

class EarthCenteredOrbit : public Orbit
{
public:
  EarthCenteredOrbit(char* tle1, char* tle2, int wgs, double current_jd); //地球周回軌道用コンストラクタ

  virtual void Propagate(double endtime, double current_jd); //軌道のプロパゲーション

  Vector<3> GetESIOmega();

  virtual string GetLogHeader() const;
  virtual string GetLogValue() const;

private:

};
