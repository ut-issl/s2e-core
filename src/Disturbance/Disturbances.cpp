#include "Disturbances.h"
#include "AirDrag.h"
#include "GGDist.h"
#include "MagDisturbance.h"
#include "SolarRadiation.h"
#include "GeoPotential.h"
#include "ThirdBodyGravity.h"
#include "../Interface/InitInput/Initialize.h"

Disturbances::Disturbances(string base_ini_fname)
  : base_ini_fname_(base_ini_fname)
{
  InitializeInstances();
  InitializeOutput();
}

Disturbances::~Disturbances()
{
  for (auto dist : disturbances_)
  {
    delete dist;
  }

  for (auto acc_dist : acc_disturbances_)
  {
    delete acc_dist;
  }
}

void Disturbances::Update(Envir & env, const Spacecraft & spacecraft)
{
  InitializeOutput();

  for (auto dist : disturbances_)
  {
    dist->UpdateIfEnabled(env, spacecraft);
    sum_torque_ += dist->GetTorque();
    sum_force_ += dist->GetForce();
  }
  sum_acceleration_i_ *= 0;
  for (auto acc_dist : acc_disturbances_)
  {
    acc_dist->UpdateIfEnabled(env, spacecraft);
    sum_acceleration_i_ += acc_dist->GetAccelerationI();
  }
}

void Disturbances::LogSetup(Logger& logger)
{
  for (auto dist : disturbances_)
  {
    logger.AddLoggable(dist);
  }
  for (auto acc_dist : acc_disturbances_)
  {
    logger.AddLoggable(acc_dist);
  }
  //Log ini file
  logger.CopyFileToLogDir(ini_fname_);
}

Vector<3> Disturbances::GetTorque()
{
  return sum_torque_;
}

Vector<3> Disturbances::GetForce()
{
  return sum_force_;
}

Vector<3> Disturbances::GetAccelerationI()
{
  return sum_acceleration_i_;
}

void Disturbances::InitializeInstances()
{
  IniAccess iniAccess = IniAccess(base_ini_fname_);
  ini_fname_ = iniAccess.ReadString("SIM_SETTING", "dist_file");

  GGDist* gg_dist = new GGDist(InitGGDist(ini_fname_));
  AirDrag* air_dist = new AirDrag(InitAirDrag(ini_fname_));
  MagDisturbance* mag_dist = new MagDisturbance(InitMagDisturbance(ini_fname_));
  SolarRadiation* srp_dist = new SolarRadiation(InitSRDist(ini_fname_));

  disturbances_.push_back(gg_dist);
  disturbances_.push_back(air_dist);
  disturbances_.push_back(mag_dist);
  disturbances_.push_back(srp_dist);

  GeoPotential* geopotential = new GeoPotential(InitGeoPotential(ini_fname_));
  ini_fname_celes_ = iniAccess.ReadString("SIM_SETTING", "celestial_file");
  ThirdBodyGravity* thirdbodygravity = new ThirdBodyGravity(InitThirdBodyGravity(ini_fname_, ini_fname_celes_));

  acc_disturbances_.push_back(geopotential);
  acc_disturbances_.push_back(thirdbodygravity);
}

void Disturbances::InitializeOutput()
{
  sum_torque_ = Vector<3>(0);
  sum_force_ = Vector<3>(0);
  sum_acceleration_i_ = Vector<3>(0);
}
