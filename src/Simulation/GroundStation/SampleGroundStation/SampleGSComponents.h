#pragma once
#include "../../../Interface/InitInput/Initialize.h"
#include "../../../Component/CommGS/ANT.h"
#include "../../../Component/CommGS/GScalculator.h"


class ANT;
class GScalculator;

class SampleGSComponents
{
public:
  SampleGSComponents(const SimulationConfig* config);
  ~SampleGSComponents();
  void CompoLogSetUp(Logger& logger);
  ANT* ant_;
  GScalculator* gscalculator_;

private:
  const SimulationConfig* config_;
};
