/**
 * @file local_environment.cpp
 * @brief Class to manage local environments
 */
#include "local_environment.hpp"

#include "dynamics/attitude/attitude.hpp"
#include "dynamics/orbit/orbit.hpp"
#include "initialize_local_environment.hpp"
#include "library/initialize/initialize_file_access.hpp"

LocalEnvironment::LocalEnvironment(SimulationConfig* simulation_configuration, const GlobalEnvironment* global_environment, const int spacecraft_id) {
  Initialize(simulation_configuration, global_environment, spacecraft_id);
}

LocalEnvironment::~LocalEnvironment() {
  delete geomagnetic_field_;
  delete solar_radiation_pressure_environment_;
  delete atmosphere_;
  delete celestial_information_;
}

void LocalEnvironment::Initialize(SimulationConfig* simulation_configuration, const GlobalEnvironment* global_environment, const int spacecraft_id) {
  // Read file name
  IniAccess iniAccess = IniAccess(simulation_configuration->sat_file_[spacecraft_id]);
  std::string ini_fname = iniAccess.ReadString("SETTING_FILES", "local_environment_file");

  // Save ini file
  simulation_configuration->main_logger_->CopyFileToLogDir(ini_fname);

  // Initialize
  geomagnetic_field_ = new GeomagneticField(InitGeomagneticField(ini_fname));
  atmosphere_ = new Atmosphere(InitAtmosphere(ini_fname));
  celestial_information_ = new LocalCelestialInformation(&(global_environment->GetCelestialInformation()));
  solar_radiation_pressure_environment_ = new SRPEnvironment(InitSRPEnvironment(ini_fname, celestial_information_));

  // Force to disable when the center body is not the Earth
  if (global_environment->GetCelestialInformation().GetCenterBodyName() != "EARTH") {
    geomagnetic_field_->IsCalcEnabled = false;
    atmosphere_->IsCalcEnabled = false;
  }

  // Log setting for Local celestial information
  IniAccess conf = IniAccess(ini_fname);
  celestial_information_->IsLogEnabled = conf.ReadEnable("LOCAL_CELESTIAL_INFORMATION", "logging");
}

void LocalEnvironment::Update(const Dynamics* dynamics, const SimTime* simulation_time) {
  auto& orbit = dynamics->GetOrbit();
  auto& attitude = dynamics->GetAttitude();

  // Update local environments that depend on the attitude (and the position)
  if (simulation_time->GetAttitudePropagateFlag()) {
    celestial_information_->UpdateAllObjectsInformation(orbit.GetSatPosition_i(), orbit.GetSatVelocity_i(), attitude.GetQuaternion_i2b(),
                                                        attitude.GetOmega_b());
    geomagnetic_field_->CalcMagneticField(simulation_time->GetCurrentDecimalYear(), simulation_time->GetCurrentSiderealTime(),
                                          orbit.GetGeodeticPosition(), attitude.GetQuaternion_i2b());
  }

  // Update local environments that depend only on the position
  if (simulation_time->GetOrbitPropagateFlag()) {
    solar_radiation_pressure_environment_->UpdateAllStates();
    atmosphere_->CalcAirDensity_kg_m3(simulation_time->GetCurrentDecimalYear(), simulation_time->GetEndTime_s(), orbit.GetGeodeticPosition());
  }
}

void LocalEnvironment::LogSetup(Logger& logger) {
  logger.AddLoggable(geomagnetic_field_);
  logger.AddLoggable(solar_radiation_pressure_environment_);
  logger.AddLoggable(atmosphere_);
  logger.AddLoggable(celestial_information_);
}
