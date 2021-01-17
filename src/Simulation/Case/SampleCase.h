#pragma once

#include "./SimulationCase.h"
#include "../Spacecraft/SampleSpacecraft/SampleSat.h"

class SampleCase : public SimulationCase
{
public:
  SampleCase(string ini_base);
  virtual ~SampleCase();
  
  void Initialize();
  void Main();

  // Log for Monte Carlo Simulation
  virtual string GetLogHeader() const;
  virtual string GetLogValue() const;

private:
  SampleSat* sample_sat_;
};
