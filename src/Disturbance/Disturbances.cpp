#include "Disturbances.h"

#include <Interface/InitInput/IniAccess.h>

#include "AirDrag.h"
#include "GGDist.h"
#include "GeoPotential.h"
#include "InitDisturbance.hpp"
#include "MagDisturbance.h"
#include "SolarRadiation.h"
#include "ThirdBodyGravity.h"

Disturbances::Disturbances(SimulationConfig* sim_config, const int sat_id, Structure* structure) {
  InitializeInstances(sim_config, sat_id, structure);
  InitializeForceAndTorque();
  InitializeAcceleration();
}

Disturbances::~Disturbances() {
  for (auto dist : disturbances_) {
    delete dist;
  }

  for (auto acc_dist : acc_disturbances_) {
    delete acc_dist;
  }
}

void Disturbances::Update(const LocalEnvironment& local_env, const Dynamics& dynamics, const SimTime* sim_time) {
  // Update disturbances that depend on the attitude (and the position)
  if (sim_time->GetAttitudePropagateFlag()) {
    InitializeForceAndTorque();
    for (auto dist : disturbances_) {
      dist->UpdateIfEnabled(local_env, dynamics);
      sum_torque_ += dist->GetTorque();
      sum_force_ += dist->GetForce();
    }
  }
  // Update disturbances that depend only on the position
  if (sim_time->GetOrbitPropagateFlag()) {
    InitializeAcceleration();
    for (auto acc_dist : acc_disturbances_) {
      acc_dist->UpdateIfEnabled(local_env, dynamics);
      sum_acceleration_i_ += acc_dist->GetAccelerationI();
    }
  }
}

void Disturbances::LogSetup(Logger& logger) {
  for (auto dist : disturbances_) {
    logger.AddLoggable(dist);
  }
  for (auto acc_dist : acc_disturbances_) {
    logger.AddLoggable(acc_dist);
  }
  // Log ini file
  logger.CopyFileToLogDir(ini_fname_);
}

Vector<3> Disturbances::GetTorque() { return sum_torque_; }

Vector<3> Disturbances::GetForce() { return sum_force_; }

Vector<3> Disturbances::GetAccelerationI() { return sum_acceleration_i_; }

void Disturbances::InitializeInstances(SimulationConfig* sim_config, const int sat_id, Structure* structure) {
  IniAccess iniAccess = IniAccess(sim_config->sat_file_[sat_id]);
  ini_fname_ = iniAccess.ReadString("DISTURBANCE", "dist_file");

  GGDist* gg_dist = new GGDist(InitGGDist(ini_fname_));
  AirDrag* air_dist = new AirDrag(InitAirDrag(ini_fname_, structure->GetSurfaces(), structure->GetKinematicsParams().GetCGb()));
  SolarRadiation* srp_dist = new SolarRadiation(InitSRDist(ini_fname_, structure->GetSurfaces(), structure->GetKinematicsParams().GetCGb()));
  MagDisturbance* mag_dist = new MagDisturbance(InitMagDisturbance(ini_fname_, structure->GetRMMParams()));

  disturbances_.push_back(gg_dist);
  disturbances_.push_back(air_dist);
  disturbances_.push_back(mag_dist);
  disturbances_.push_back(srp_dist);

  GeoPotential* geopotential = new GeoPotential(InitGeoPotential(ini_fname_));
  ThirdBodyGravity* thirdbodygravity = new ThirdBodyGravity(InitThirdBodyGravity(ini_fname_, sim_config->ini_base_fname_));

  acc_disturbances_.push_back(geopotential);
  acc_disturbances_.push_back(thirdbodygravity);
}

void Disturbances::InitializeForceAndTorque() {
  sum_torque_ = Vector<3>(0);
  sum_force_ = Vector<3>(0);
}

void Disturbances::InitializeAcceleration() { sum_acceleration_i_ = Vector<3>(0); }