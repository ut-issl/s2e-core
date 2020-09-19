#pragma once
#include "../../../Library/math/Vector.hpp"
#include "../../../Dynamics/Dynamics.h"
#include "../../../Component/CDH/OBC.h"
#include "../../../Component/AOCS/Gyro.h"

using libra::Vector;
class OBC;
class Gyro;

class SampleComponents
{
public:
  SampleComponents(const Dynamics* dynamics, const SimulationConfig* config, ClockGenerator* clock_gen, const int sat_id);
  ~SampleComponents();
  Vector<3> GenerateForce_b();
  Vector<3> GenerateTorque_b();
  void CompoLogSetUp(Logger& logger);
private:
  OBC* obc_;
  Gyro* gyro_;
  const SimulationConfig* config_;
  const Dynamics* dynamics_;
};
