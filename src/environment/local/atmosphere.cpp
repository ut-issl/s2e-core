/**
 * @file atmosphere.cpp
 * @brief Class to calculate earth's atmospheric density
 */

#include "atmosphere.hpp"

#include "logger/log_utility.hpp"
#include "math_physics/atmosphere/harris_priester_model.hpp"
#include "math_physics/atmosphere/simple_air_density_model.hpp"
#include "math_physics/math/vector.hpp"
#include "math_physics/randomization/global_randomization.hpp"
#include "math_physics/randomization/normal_randomization.hpp"
#include "math_physics/randomization/random_walk.hpp"
#include "setting_file_reader/initialize_file_access.hpp"

namespace s2e::environment {

Atmosphere::Atmosphere(const std::string model, const std::string space_weather_file_name, const double gauss_standard_deviation_rate,
                       const bool is_manual_param, const double manual_f107, const double manual_f107a, const double manual_ap,
                       const LocalCelestialInformation* local_celestial_information, const SimulationTime* simulation_time)
    : model_(model),
      air_density_kg_m3_(0.0),
      is_manual_param_used_(is_manual_param),
      manual_daily_f107_(manual_f107),
      manual_average_f107_(manual_f107a),
      manual_ap_(manual_ap),
      gauss_standard_deviation_rate_(gauss_standard_deviation_rate),
      local_celestial_information_(local_celestial_information) {
  if (model_ == "STANDARD") {
    // Standard
    std::cerr << "Air density model : STANDARD" << std::endl;
  } else if (model_ == "NRLMSISE00") {
    // NRLMSISE-00
    std::cerr << "Air density model : NRLMSISE00" << std::endl;
    if (!is_manual_param_used_) {
      double decimal_year = simulation_time->GetCurrentDecimalYear();
      double end_time_s = simulation_time->GetEndTime_s();
      if (GetSpaceWeatherTable_(decimal_year, end_time_s, space_weather_file_name, space_weather_table_)) {
      } else {
        std::cerr << "Space Weather file read error!" << std::endl;
        std::cerr << "Air density is switched to STANDARD model" << std::endl;
        model_ = "STANDARD";
      }
    }
  } else if (model_ == "HARRIS_PRIESTER") {
    // Harris-Priester
    std::cerr << "Air density model : Harris-Priester" << std::endl;
  } else {
    std::cerr << "Air density model : None" << std::endl;
    std::cerr << "Air density is set as 0.0 kg/m3" << std::endl;
  }
}

double Atmosphere::CalcAirDensity_kg_m3(const double decimal_year, const dynamics::orbit::Orbit& orbit) {
  if (!is_calc_enabled_) return 0;

  if (model_ == "STANDARD") {
    // Standard model
    double altitude_m = orbit.GetGeodeticPosition().GetAltitude_m();
    air_density_kg_m3_ = s2e::atmosphere::CalcAirDensityWithSimpleModel(altitude_m);
  } else if (model_ == "NRLMSISE00") {
    // NRLMSISE00 model
    double lat_rad = orbit.GetGeodeticPosition().GetLatitude_rad();
    double lon_rad = orbit.GetGeodeticPosition().GetLongitude_rad();
    double alt_m = orbit.GetGeodeticPosition().GetAltitude_m();
    air_density_kg_m3_ = CalcNRLMSISE00(decimal_year, lat_rad, lon_rad, alt_m, space_weather_table_, is_manual_param_used_, manual_daily_f107_,
                                        manual_average_f107_, manual_ap_);
  } else if (model_ == "HARRIS_PRIESTER") {
    // Harris-Priester
    math::Vector<3> sun_direction_eci = local_celestial_information_->GetGlobalInformation().GetPositionFromCenter_i_m("SUN").CalcNormalizedVector();
    air_density_kg_m3_ = s2e::atmosphere::CalcAirDensityWithHarrisPriester_kg_m3(orbit.GetGeodeticPosition(), sun_direction_eci);
  } else {
    // No suitable model
    return air_density_kg_m3_ = 0.0;
  }

  return AddNoise(air_density_kg_m3_);
}

double Atmosphere::AddNoise(const double rho_kg_m3) {
  // RandomWalk rw(rho_kg_m3*rw_stepwidth_,rho_kg_m3*rw_stddev_,rho_kg_m3*rw_limit_);
  s2e::randomization::NormalRand nr(0.0, rho_kg_m3 * gauss_standard_deviation_rate_, s2e::randomization::global_randomization.MakeSeed());
  double nrd = nr;

  return rho_kg_m3 + nrd;
}

std::string Atmosphere::GetLogValue() const {
  std::string str_tmp = "";
  str_tmp += logger::WriteScalar(air_density_kg_m3_);

  return str_tmp;
}

std::string Atmosphere::GetLogHeader() const {
  std::string str_tmp = "";

  str_tmp += logger::WriteScalar("air_density_at_spacecraft_position", "kg/m3");

  return str_tmp;
}

Atmosphere InitAtmosphere(const std::string initialize_file_path, const LocalCelestialInformation* local_celestial_information,
                          const SimulationTime* simulation_time) {
  auto conf = setting_file_reader::IniAccess(initialize_file_path);
  const char* section = "ATMOSPHERE";
  double f107_threshold = 50.0;
  double f107_default = 150.0;

  std::string model = conf.ReadString(section, "model");
  std::string table_path = conf.ReadString(section, "nrlmsise00_table_file");
  double rho_stddev = conf.ReadDouble(section, "air_density_standard_deviation");
  bool is_manual_param_used = conf.ReadEnable(section, "is_manual_parameter_used");
  double manual_daily_f107 = conf.ReadDouble(section, "manual_daily_f107");
  if (manual_daily_f107 < f107_threshold) {
    std::cerr << "Daily F10.7 may be too low. It is set as 150.0 in this simulation. "
                 "Check [ATMOSPHERE] section in LocalEnvironment.ini."
              << std::endl;
    manual_daily_f107 = f107_default;
  }
  double manual_average_f107 = conf.ReadDouble(section, "manual_average_f107");
  if (manual_average_f107 < f107_threshold) {
    std::cerr << "Average F10.7 may be too low. It is set as 150.0 in this "
                 "simulation. Check [ATMOSPHERE] section in LocalEnvironment.ini."
              << std::endl;
    manual_average_f107 = f107_default;
  }
  double manual_ap = conf.ReadDouble(section, "manual_ap");

  Atmosphere atmosphere(model, table_path, rho_stddev, is_manual_param_used, manual_daily_f107, manual_average_f107, manual_ap,
                        local_celestial_information, simulation_time);
  atmosphere.SetCalcFlag(conf.ReadEnable(section, INI_CALC_LABEL));
  atmosphere.is_log_enabled_ = conf.ReadEnable(section, INI_LOG_LABEL);

  return atmosphere;
}

}  // namespace s2e::environment
