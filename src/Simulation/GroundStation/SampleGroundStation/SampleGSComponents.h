#pragma once
#include <Interface/InitInput/Initialize.h>

#include <Component/CommGS/InitAnt.hpp>
#include <Component/CommGS/InitGsCalculator.hpp>

class ANT;
class GScalculator;

class SampleGSComponents {
 public:
  SampleGSComponents(const SimulationConfig* config);
  ~SampleGSComponents();
  void CompoLogSetUp(Logger& logger);
  ANT* ant_;
  GScalculator* gscalculator_;

 private:
  const SimulationConfig* config_;
};
