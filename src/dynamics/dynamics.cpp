/**
 * @file dynamics.cpp
 * @brief Class to manage dynamics of spacecraft
 */

#include "dynamics.hpp"

#include "../simulation/multiple_spacecraft/relative_information.hpp"

Dynamics::Dynamics(SimulationConfig* simulation_configuration, const SimulationTime* simulation_time,
                   const LocalCelestialInformation* local_celestial_information, const int spacecraft_id, Structure* structure,
                   RelativeInformation* relative_information) {
  Initialize(simulation_configuration, simulation_time, local_celestial_information, spacecraft_id, structure, relative_information);
}

Dynamics::~Dynamics() {
  delete attitude_;
  delete orbit_;
  delete temperature_;
}

void Dynamics::Initialize(SimulationConfig* simulation_configuration, const SimulationTime* simulation_time,
                          const LocalCelestialInformation* local_celestial_information, const int spacecraft_id, Structure* structure,
                          RelativeInformation* relative_information) {
  structure_ = structure;

  // Initialize
  orbit_ = InitOrbit(&(local_celestial_information->GetGlobalInformation()), simulation_configuration->sat_file_[spacecraft_id],
                     simulation_time->GetOrbitRkStepTime_s(), simulation_time->GetCurrentTime_jd(),
                     local_celestial_information->GetGlobalInformation().GetCenterBodyGravityConstant_m3_s2(), "ORBIT", relative_information);
  attitude_ = InitAttitude(simulation_configuration->sat_file_[spacecraft_id], orbit_, local_celestial_information,
                           simulation_time->GetAttitudeRkStepTime_s(), structure->GetKinematicsParams().GetInertiaTensor(), spacecraft_id);
  temperature_ = InitTemperature(simulation_configuration->sat_file_[spacecraft_id], simulation_time->GetThermalRkStepTime_s());

  // To get initial value
  orbit_->UpdateAtt(attitude_->GetQuaternion_i2b());
}

void Dynamics::Update(const SimulationTime* simulation_time, const LocalCelestialInformation* local_celestial_information) {
  // Attitude propagation
  if (simulation_time->GetAttitudePropagateFlag()) {
    attitude_->Propagate(simulation_time->GetElapsedTime_s());
  }
  // Orbit Propagation
  if (simulation_time->GetOrbitPropagateFlag()) {
    orbit_->Propagate(simulation_time->GetElapsedTime_s(), simulation_time->GetCurrentTime_jd());
  }
  // Attitude dependent update
  orbit_->UpdateAtt(attitude_->GetQuaternion_i2b());

  // Thermal
  if (simulation_time->GetThermalPropagateFlag()) {
    std::string sun_str = "SUN";
    char* c_sun = new char[sun_str.size() + 1];
    std::char_traits<char>::copy(c_sun, sun_str.c_str(), sun_str.size() + 1);  // string -> char*
    temperature_->Propagate(local_celestial_information->GetPositionFromSpacecraft_b_m(c_sun), simulation_time->GetElapsedTime_s());
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
