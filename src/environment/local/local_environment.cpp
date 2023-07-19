/**
 * @file local_environment.cpp
 * @brief Class to manage local environments
 */
#include "local_environment.hpp"

#include "dynamics/attitude/attitude.hpp"
#include "dynamics/orbit/orbit.hpp"
#include "initialize_local_environment.hpp"
#include "library/initialize/initialize_file_access.hpp"

LocalEnvironment::LocalEnvironment(const SimulationConfiguration* simulation_configuration, const GlobalEnvironment* global_environment,
                                   const int spacecraft_id) {
  Initialize(simulation_configuration, global_environment, spacecraft_id);
}

LocalEnvironment::~LocalEnvironment() {
  delete geomagnetic_field_;
  delete solar_radiation_pressure_environment_;
  delete atmosphere_;
  delete celestial_information_;
}

void LocalEnvironment::Initialize(const SimulationConfiguration* simulation_configuration, const GlobalEnvironment* global_environment,
                                  const int spacecraft_id) {
  // Read file name
  IniAccess iniAccess = IniAccess(simulation_configuration->spacecraft_file_list_[spacecraft_id]);
  std::string ini_fname = iniAccess.ReadString("SETTING_FILES", "local_environment_file");

  // Save ini file
  simulation_configuration->main_logger_->CopyFileToLogDirectory(ini_fname);

  // Initialize
  geomagnetic_field_ = new GeomagneticField(InitGeomagneticField(ini_fname));
  celestial_information_ = new LocalCelestialInformation(&(global_environment->GetCelestialInformation()));
  atmosphere_ = new Atmosphere(InitAtmosphere(ini_fname, celestial_information_, &global_environment->GetSimulationTime()));
  solar_radiation_pressure_environment_ =
      new SolarRadiationPressureEnvironment(InitSolarRadiationPressureEnvironment(ini_fname, celestial_information_));

  // Force to disable when the center body is not the Earth
  if (global_environment->GetCelestialInformation().GetCenterBodyName() != "EARTH") {
    geomagnetic_field_->IsCalcEnabled = false;
    atmosphere_->SetCalcFlag(false);
  }

  // Log setting for Local celestial information
  IniAccess conf = IniAccess(ini_fname);
  celestial_information_->is_log_enabled_ = conf.ReadEnable("LOCAL_CELESTIAL_INFORMATION", "logging");
}

void LocalEnvironment::Update(const Dynamics* dynamics, const SimulationTime* simulation_time) {
  auto& orbit = dynamics->GetOrbit();
  auto& attitude = dynamics->GetAttitude();

  // Update local environments that depend on the attitude (and the position)
  if (simulation_time->GetAttitudePropagateFlag()) {
    celestial_information_->UpdateAllObjectsInformation(orbit.GetPosition_i_m(), orbit.GetVelocity_i_m_s(), attitude.GetQuaternion_i2b(),
                                                        attitude.GetAngularVelocity_b_rad_s());
    geomagnetic_field_->CalcMagneticField(simulation_time->GetCurrentDecimalYear(), simulation_time->GetCurrentSiderealTime(),
                                          orbit.GetGeodeticPosition(), attitude.GetQuaternion_i2b());
  }

  // Update local environments that depend only on the position
  if (simulation_time->GetOrbitPropagateFlag()) {
    solar_radiation_pressure_environment_->UpdateAllStates();
    atmosphere_->CalcAirDensity_kg_m3(simulation_time->GetCurrentDecimalYear(), orbit);
  }
}

void LocalEnvironment::LogSetup(Logger& logger) {
  logger.AddLogList(geomagnetic_field_);
  logger.AddLogList(solar_radiation_pressure_environment_);
  logger.AddLogList(atmosphere_);
  logger.AddLogList(celestial_information_);
}
