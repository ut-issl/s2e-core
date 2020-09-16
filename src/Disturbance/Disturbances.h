#pragma once

#include <vector>
#include "SimpleDisturbance.h"
#include "AccelerationDisturbance.h"

using namespace std;

class Logger;

class Disturbances
{
public:
  Disturbances(string base_ini_fname,const vector<Surface>& surfaces);
  virtual ~Disturbances();
  void Update(const LocalEnvironment& local_env, const Dynamics& dynamics);

  void LogSetup(Logger & logger);

  Vector<3> GetTorque();
  Vector<3> GetForce();
  Vector<3> GetAccelerationI();

private:
  string ini_fname_;
  string base_ini_fname_;
  string ini_fname_celes_;
  void InitializeInstances(const vector<Surface>& surfaces);
  void InitializeOutput();
  vector<SimpleDisturbance*> disturbances_;
  Vector<3> sum_torque_;
  Vector<3> sum_force_;
  vector<AccelerationDisturbance*> acc_disturbances_;
  Vector<3> sum_acceleration_i_;
};
