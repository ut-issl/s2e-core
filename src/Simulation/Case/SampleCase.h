#pragma once

#include "../GroundStation/GroundStation.h"
#include "../Spacecraft/SampleSpacecraft/SampleSat.h"
#include "./SimulationCase.h"

class SampleCase : public SimulationCase {
 public:
  SampleCase(std::string ini_base);
  virtual ~SampleCase();

  void Initialize();
  void Main();

  // Log for Monte Carlo Simulation
  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;

 private:
  SampleSat* sample_sat_;
  GroundStation* sample_gs_;
};
