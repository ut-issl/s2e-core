/**
 * @file dynamics.cpp
 * @brief Class to manage dynamics of spacecraft
 */

#include "dynamics.hpp"

#include "../simulation/multiple_spacecraft/relative_information.hpp"

Dynamics::Dynamics(SimulationConfig* sim_config, const SimulationTime* sim_time, const LocalCelestialInformation* local_celes_info, const int sat_id,
                   Structure* structure, RelativeInformation* rel_info) {
  Initialize(sim_config, sim_time, local_celes_info, sat_id, structure, rel_info);
}

Dynamics::~Dynamics() {
  delete attitude_;
  delete orbit_;
  delete temperature_;
}

void Dynamics::Initialize(SimulationConfig* sim_config, const SimulationTime* sim_time, const LocalCelestialInformation* local_celes_info,
                          const int sat_id, Structure* structure, RelativeInformation* rel_info) {
  structure_ = structure;

  // Initialize
  orbit_ = InitOrbit(&(local_celes_info->GetGlobalInformation()), sim_config->sat_file_[sat_id], sim_time->GetOrbitRkStepTime_s(),
                     sim_time->GetCurrentTime_jd(), local_celes_info->GetGlobalInformation().GetCenterBodyGravityConstant_m3_s2(), "ORBIT", rel_info);
  attitude_ = InitAttitude(sim_config->sat_file_[sat_id], orbit_, local_celes_info, sim_time->GetAttitudeRkStepTime_s(),
                           structure->GetKinematicsParams().GetInertiaTensor(), sat_id);
  temperature_ = InitTemperature(sim_config->sat_file_[sat_id], sim_time->GetThermalRkStepTime_s());

  // To get initial value
  orbit_->UpdateAtt(attitude_->GetQuaternion_i2b());
}

void Dynamics::Update(const SimulationTime* sim_time, const LocalCelestialInformation* local_celes_info) {
  // Attitude propagation
  if (sim_time->GetAttitudePropagateFlag()) {
    attitude_->Propagate(sim_time->GetElapsedTime_s());
  }
  // Orbit Propagation
  if (sim_time->GetOrbitPropagateFlag()) {
    orbit_->Propagate(sim_time->GetElapsedTime_s(), sim_time->GetCurrentTime_jd());
  }
  // Attitude dependent update
  orbit_->UpdateAtt(attitude_->GetQuaternion_i2b());

  // Thermal
  if (sim_time->GetThermalPropagateFlag()) {
    std::string sun_str = "SUN";
    char* c_sun = new char[sun_str.size() + 1];
    std::char_traits<char>::copy(c_sun, sun_str.c_str(), sun_str.size() + 1);  // string -> char*
    temperature_->Propagate(local_celes_info->GetPositionFromSpacecraft_b_m(c_sun), sim_time->GetElapsedTime_s());
    delete[] c_sun;
  }
}

void Dynamics::ClearForceTorque(void) {
  libra::Vector<3> zero(0.0);
  attitude_->SetTorque_b(zero);
  orbit_->SetAcceleration_i(zero);
}

void Dynamics::LogSetup(Logger& logger) {
  logger.AddLoggable(attitude_);
  logger.AddLoggable(orbit_);
}

void Dynamics::AddTorque_b(libra::Vector<3> torque_b) { attitude_->AddTorque_b(torque_b); }

void Dynamics::AddForce_b(libra::Vector<3> force_b) {
  orbit_->AddForce_b(force_b, attitude_->GetQuaternion_i2b(), structure_->GetKinematicsParams().GetMass());
}

void Dynamics::AddAcceleration_i(libra::Vector<3> acceleration_i) { orbit_->AddAcceleration_i(acceleration_i); }

libra::Vector<3> Dynamics::GetPosition_i() const { return orbit_->GetSatPosition_i(); }

libra::Quaternion Dynamics::GetQuaternion_i2b() const { return attitude_->GetQuaternion_i2b(); }
