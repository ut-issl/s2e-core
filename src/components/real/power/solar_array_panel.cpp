/*
 * @file solar_array_panel.cpp
 * @brief Component emulation of Solar Array Panel
 */

#include "solar_array_panel.hpp"

#include <components/real/power/csv_scenario_interface.hpp>
#include <environment/global/clock_generator.hpp>
#include <initial_setting_file/initialize_file_access.hpp>

SolarArrayPanel::SolarArrayPanel(const int prescaler, ClockGenerator* clock_generator, int component_id, int number_of_series, int number_of_parallel,
                                 double cell_area_m2, libra::Vector<3> normal_vector, double cell_efficiency, double transmission_efficiency,
                                 const SolarRadiationPressureEnvironment* srp_environment,
                                 const LocalCelestialInformation* local_celestial_information, double component_step_time_s)
    : Component(prescaler, clock_generator),
      component_id_(component_id),
      number_of_series_(number_of_series),
      number_of_parallel_(number_of_parallel),
      cell_area_m2_(cell_area_m2),
      normal_vector_(normal_vector.CalcNormalizedVector()),
      cell_efficiency_(cell_efficiency),
      transmission_efficiency_(transmission_efficiency),
      srp_environment_(srp_environment),
      local_celestial_information_(local_celestial_information),
      compo_step_time_s_(component_step_time_s) {
  voltage_V_ = 0.0;
  power_generation_W_ = 0.0;
}

SolarArrayPanel::SolarArrayPanel(const int prescaler, ClockGenerator* clock_generator, int component_id, int number_of_series, int number_of_parallel,
                                 double cell_area_m2, libra::Vector<3> normal_vector, double cell_efficiency, double transmission_efficiency,
                                 const SolarRadiationPressureEnvironment* srp_environment, double component_step_time_s)
    : Component(prescaler, clock_generator),
      component_id_(component_id),
      number_of_series_(number_of_series),
      number_of_parallel_(number_of_parallel),
      cell_area_m2_(cell_area_m2),
      normal_vector_(normal_vector.CalcNormalizedVector()),
      cell_efficiency_(cell_efficiency),
      transmission_efficiency_(transmission_efficiency),
      srp_environment_(srp_environment),
      compo_step_time_s_(component_step_time_s) {
  voltage_V_ = 0.0;
  power_generation_W_ = 0.0;
}

SolarArrayPanel::SolarArrayPanel(ClockGenerator* clock_generator, int component_id, int number_of_series, int number_of_parallel, double cell_area_m2,
                                 libra::Vector<3> normal_vector, double cell_efficiency, double transmission_efficiency,
                                 const SolarRadiationPressureEnvironment* srp_environment,
                                 const LocalCelestialInformation* local_celestial_information)
    : Component(10, clock_generator),
      component_id_(component_id),
      number_of_series_(number_of_series),
      number_of_parallel_(number_of_parallel),
      cell_area_m2_(cell_area_m2),
      normal_vector_(normal_vector.CalcNormalizedVector()),
      cell_efficiency_(cell_efficiency),
      transmission_efficiency_(transmission_efficiency),
      srp_environment_(srp_environment),
      local_celestial_information_(local_celestial_information),
      compo_step_time_s_(0.1) {
  voltage_V_ = 0.0;
  power_generation_W_ = 0.0;
}

SolarArrayPanel::SolarArrayPanel(const SolarArrayPanel& obj)
    : Component(obj),
      component_id_(obj.component_id_),
      number_of_series_(obj.number_of_series_),
      number_of_parallel_(obj.number_of_parallel_),
      cell_area_m2_(obj.cell_area_m2_),
      normal_vector_(obj.normal_vector_),
      cell_efficiency_(obj.cell_efficiency_),
      transmission_efficiency_(obj.transmission_efficiency_),
      srp_environment_(obj.srp_environment_),
      local_celestial_information_(obj.local_celestial_information_),
      compo_step_time_s_(obj.compo_step_time_s_) {
  voltage_V_ = 0.0;
  power_generation_W_ = 0.0;
}

SolarArrayPanel::~SolarArrayPanel() {}

std::string SolarArrayPanel::GetLogHeader() const {
  std::string str_tmp = "";
  std::string component_name = "sap" + std::to_string(component_id_) + "_";
  str_tmp += WriteScalar(component_name + "generated_power", "W");
  return str_tmp;
}

std::string SolarArrayPanel::GetLogValue() const {
  std::string str_tmp = "";
  str_tmp += WriteScalar(power_generation_W_);
  return str_tmp;
}

void SolarArrayPanel::MainRoutine(const int time_count) {
  if (CsvScenarioInterface::IsCsvScenarioEnabled()) {
    double time_query = compo_step_time_s_ * time_count;
    const auto solar_constant = srp_environment_->GetSolarConstant_W_m2();
    libra::Vector<3> sun_direction_body = CsvScenarioInterface::GetSunDirectionBody(time_query);
    libra::Vector<3> normalized_sun_direction_body = sun_direction_body.CalcNormalizedVector();
    power_generation_W_ = cell_efficiency_ * transmission_efficiency_ * solar_constant * (int)CsvScenarioInterface::GetSunFlag(time_query) *
                          cell_area_m2_ * number_of_parallel_ * number_of_series_ * InnerProduct(normal_vector_, normalized_sun_direction_body);
  } else {
    const auto power_density = srp_environment_->GetPowerDensity_W_m2();
    libra::Vector<3> sun_pos_b = local_celestial_information_->GetPositionFromSpacecraft_b_m("SUN");
    libra::Vector<3> sun_dir_b = sun_pos_b.CalcNormalizedVector();
    power_generation_W_ = cell_efficiency_ * transmission_efficiency_ * power_density * cell_area_m2_ * number_of_parallel_ * number_of_series_ *
                          InnerProduct(normal_vector_, sun_dir_b);
    // TODO: Improve implementation. For example, update IV curve with sun direction and calculate generated power
  }
  if (power_generation_W_ < 0) power_generation_W_ = 0.0;
}

SolarArrayPanel InitSAP(ClockGenerator* clock_generator, int sap_id, const std::string file_name,
                        const SolarRadiationPressureEnvironment* srp_environment, const LocalCelestialInformation* local_celestial_information,
                        double component_step_time_s) {
  IniAccess sap_conf(file_name);

  const std::string section_name = "SOLAR_ARRAY_PANEL_" + std::to_string(static_cast<long long>(sap_id));

  int prescaler = sap_conf.ReadInt(section_name.c_str(), "prescaler");
  if (prescaler <= 1) prescaler = 1;

  int number_of_series;
  number_of_series = sap_conf.ReadInt(section_name.c_str(), "number_of_series");

  int number_of_parallel;
  number_of_parallel = sap_conf.ReadInt(section_name.c_str(), "number_of_parallel");

  double cell_area_m2;
  cell_area_m2 = sap_conf.ReadDouble(section_name.c_str(), "cell_area_m2");

  libra::Vector<3> normal_vector;
  sap_conf.ReadVector(section_name.c_str(), "normal_vector_b", normal_vector);

  double cell_efficiency;
  cell_efficiency = sap_conf.ReadDouble(section_name.c_str(), "cell_efficiency");

  double transmission_efficiency;
  transmission_efficiency = sap_conf.ReadDouble(section_name.c_str(), "transmission_efficiency");

  SolarArrayPanel sap(prescaler, clock_generator, sap_id, number_of_series, number_of_parallel, cell_area_m2, normal_vector, cell_efficiency,
                      transmission_efficiency, srp_environment, local_celestial_information, component_step_time_s);

  return sap;
}

SolarArrayPanel InitSAP(ClockGenerator* clock_generator, int sap_id, const std::string file_name,
                        const SolarRadiationPressureEnvironment* srp_environment, double component_step_time_s) {
  IniAccess sap_conf(file_name);

  const std::string section_name = "SOLAR_ARRAY_PANEL_" + std::to_string(static_cast<long long>(sap_id));

  int prescaler = sap_conf.ReadInt(section_name.c_str(), "prescaler");
  if (prescaler <= 1) prescaler = 1;

  int number_of_series;
  number_of_series = sap_conf.ReadInt(section_name.c_str(), "number_of_series");

  int number_of_parallel;
  number_of_parallel = sap_conf.ReadInt(section_name.c_str(), "number_of_parallel");

  double cell_area_m2;
  cell_area_m2 = sap_conf.ReadDouble(section_name.c_str(), "cell_area_m2");

  libra::Vector<3> normal_vector;
  sap_conf.ReadVector(section_name.c_str(), "normal_vector_b", normal_vector);

  double cell_efficiency;
  cell_efficiency = sap_conf.ReadDouble(section_name.c_str(), "cell_efficiency");

  double transmission_efficiency;
  transmission_efficiency = sap_conf.ReadDouble(section_name.c_str(), "transmission_efficiency");

  SolarArrayPanel sap(prescaler, clock_generator, sap_id, number_of_series, number_of_parallel, cell_area_m2, normal_vector, cell_efficiency,
                      transmission_efficiency, srp_environment, component_step_time_s);

  return sap;
}
