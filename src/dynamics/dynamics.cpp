/**
 * @file dynamics.cpp
 * @brief Class to manage dynamics of spacecraft
 */

#include "dynamics.hpp"

namespace s2e::dynamics {

Dynamics::Dynamics(const simulation::SimulationConfiguration* simulation_configuration, const environment::SimulationTime* simulation_time,
                   const environment::LocalEnvironment* local_environment, const int spacecraft_id, spacecraft::Structure* structure,
                   simulation::RelativeInformation* relative_information)
    : structure_(structure), local_environment_(local_environment) {
  Initialize(simulation_configuration, simulation_time, spacecraft_id, structure, relative_information);
}

Dynamics::~Dynamics() {
  delete attitude_;
  delete orbit_;
  delete temperature_;
}

void Dynamics::Initialize(const simulation::SimulationConfiguration* simulation_configuration, const environment::SimulationTime* simulation_time,
                          const int spacecraft_id, spacecraft::Structure* structure, simulation::RelativeInformation* relative_information) {
  const environment::LocalCelestialInformation& local_celestial_information = local_environment_->GetCelestialInformation();
  // Initialize
  orbit_ = orbit::InitOrbit(&(local_celestial_information.GetGlobalInformation()), simulation_configuration->spacecraft_file_list_[spacecraft_id],
                            simulation_time->GetOrbitRkStepTime_s(), simulation_time->GetCurrentTime_jd(),
                            local_celestial_information.GetGlobalInformation().GetCenterBodyGravityConstant_m3_s2(), "ORBIT", relative_information);
  attitude_ = attitude::InitAttitude(simulation_configuration->spacecraft_file_list_[spacecraft_id], orbit_, &local_celestial_information,
                                     simulation_time->GetAttitudeRkStepTime_s(), structure->GetKinematicsParameters().GetInertiaTensor_b_kgm2(),
                                     spacecraft_id);
  temperature_ = thermal::InitTemperature(simulation_configuration->spacecraft_file_list_[spacecraft_id], simulation_time->GetThermalRkStepTime_s(),
                                          &(local_environment_->GetSolarRadiationPressure()), &(local_environment_->GetEarthAlbedo()));

  // To get initial value
  orbit_->UpdateByAttitude(attitude_->GetQuaternion_i2b());
}

void Dynamics::Update(const environment::SimulationTime* simulation_time, const environment::LocalCelestialInformation* local_celestial_information) {
  // Attitude propagation
  if (simulation_time->GetAttitudePropagateFlag()) {
    attitude_->Propagate(simulation_time->GetElapsedTime_s());
  }
  // Orbit Propagation
  if (simulation_time->GetOrbitPropagateFlag()) {
    orbit_->Propagate(simulation_time->GetElapsedTime_s(), simulation_time->GetCurrentTime_jd());
  }
  // Attitude dependent update
  orbit_->UpdateByAttitude(attitude_->GetQuaternion_i2b());

  // Thermal
  if (simulation_time->GetThermalPropagateFlag()) {
    temperature_->Propagate(local_celestial_information, simulation_time->GetElapsedTime_s());
  }
}

void Dynamics::ClearForceTorque(void) {
  math::Vector<3> zero(0.0);
  attitude_->SetTorque_b_Nm(zero);
  orbit_->SetAcceleration_i_m_s2(zero);
}

void Dynamics::LogSetup(logger::Logger& logger) {
  logger.AddLogList(attitude_);
  logger.AddLogList(orbit_);
  logger.AddLogList(temperature_);
}

}  // namespace s2e::dynamics
