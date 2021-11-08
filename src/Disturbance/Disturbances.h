#pragma once

#include <vector>
#include "SimpleDisturbance.h"
#include "AccelerationDisturbance.h"
#include"../Simulation/Spacecraft/Structure/Structure.h"
#include"../Environment/Global/SimTime.h"

class Logger;

class Disturbances
{
public:
  Disturbances(SimulationConfig* sim_config, const int sat_id, Structure* structure);
  virtual ~Disturbances();
  void Update(const LocalEnvironment& local_env, const Dynamics& dynamics, const SimTime* sim_time);

  void LogSetup(Logger & logger);

  Vector<3> GetTorque();
  Vector<3> GetForce();
  Vector<3> GetAccelerationI();

private:
  std::string ini_fname_;
  void InitializeInstances(SimulationConfig* sim_config, const int sat_id, Structure* structure);
  void InitializeForceAndTorque();
  void InitializeAcceleration();
  std::vector<SimpleDisturbance*> disturbances_;
  Vector<3> sum_torque_;
  Vector<3> sum_force_;
  vector<AccelerationDisturbance*> acc_disturbances_;
  Vector<3> sum_acceleration_i_;
};
