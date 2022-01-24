#pragma once
#include "Orbit.h"
#include "../../Library/Orbit/KeplerOrbit.h"

class KeplerOrbitPropagation : public Orbit, public KeplerOrbit
{
public:
  // Initialize with orbital elements
  KeplerOrbitPropagation(
    const double current_jd,
    KeplerOrbit kepler_orbit
  );
  ~KeplerOrbitPropagation();

  // Orbit class
  virtual void Propagate(double endtime, double current_jd);
  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;

private:
  void UpdateState(const double current_jd);
};
