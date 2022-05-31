#pragma once

#include <Component/CommGS/InitAnt.hpp>
#include <Component/CommGS/InitGsCalculator.hpp>

class SampleGSComponents {
 public:
  SampleGSComponents(const SimulationConfig* config);
  ~SampleGSComponents();
  void CompoLogSetUp(Logger& logger);

  // Getter
  inline ANT* GetAntenna() const { return antenna_; }
  inline GScalculator* GetGsCalculator() const { return gs_calculator_; }

 private:
  ANT* antenna_;
  GScalculator* gs_calculator_;
  const SimulationConfig* config_;
};
