#include "SAP.h"

#include <Component/Power/CsvScenarioInterface.h>
#include <Environment/Global/ClockGenerator.h>

SAP::SAP(const int prescaler, ClockGenerator* clock_gen, int id, int number_of_series, int number_of_parallel, double cell_area, libra::Vector<3> normal_vector,
         double cell_efficiency, double transmission_efficiency, const SRPEnvironment* srp, const LocalCelestialInformation* local_celes_info, double compo_step_time)
    : ComponentBase(prescaler, clock_gen),
      id_(id),
      number_of_series_(number_of_series),
      number_of_parallel_(number_of_parallel),
      cell_area_(cell_area),
      normal_vector_(libra::normalize(normal_vector)),
      cell_efficiency_(cell_efficiency),
      transmission_efficiency_(transmission_efficiency),
      srp_(srp),
      local_celes_info_(local_celes_info),
      compo_step_time_(compo_step_time) {
  voltage_ = 0.0;
  power_generation_ = 0.0;
}

SAP::SAP(const int prescaler, ClockGenerator* clock_gen, int id, int number_of_series, int number_of_parallel, double cell_area, libra::Vector<3> normal_vector,
         double cell_efficiency, double transmission_efficiency, const SRPEnvironment* srp, double compo_step_time)
    : ComponentBase(prescaler, clock_gen),
      id_(id),
      number_of_series_(number_of_series),
      number_of_parallel_(number_of_parallel),
      cell_area_(cell_area),
      normal_vector_(libra::normalize(normal_vector)),
      cell_efficiency_(cell_efficiency),
      transmission_efficiency_(transmission_efficiency),
      srp_(srp),
      compo_step_time_(compo_step_time) {
  voltage_ = 0.0;
  power_generation_ = 0.0;
}

SAP::SAP(ClockGenerator* clock_gen, int id, int number_of_series, int number_of_parallel, double cell_area, libra::Vector<3> normal_vector,
         double cell_efficiency, double transmission_efficiency, const SRPEnvironment* srp, const LocalCelestialInformation* local_celes_info)
    : ComponentBase(10, clock_gen),
      id_(id),
      number_of_series_(number_of_series),
      number_of_parallel_(number_of_parallel),
      cell_area_(cell_area),
      normal_vector_(libra::normalize(normal_vector)),
      cell_efficiency_(cell_efficiency),
      transmission_efficiency_(transmission_efficiency),
      srp_(srp),
      local_celes_info_(local_celes_info),
      compo_step_time_(0.1) {
  voltage_ = 0.0;
  power_generation_ = 0.0;
}

SAP::SAP(const SAP& obj)
    : ComponentBase(obj),
      id_(obj.id_),
      number_of_series_(obj.number_of_series_),
      number_of_parallel_(obj.number_of_parallel_),
      cell_area_(obj.cell_area_),
      normal_vector_(obj.normal_vector_),
      cell_efficiency_(obj.cell_efficiency_),
      transmission_efficiency_(obj.transmission_efficiency_),
      srp_(obj.srp_),
      local_celes_info_(obj.local_celes_info_),
      compo_step_time_(obj.compo_step_time_) {
  voltage_ = 0.0;
  power_generation_ = 0.0;
}

SAP::~SAP() {}

double SAP::GetPowerGeneration() const { return power_generation_; }

void SAP::SetVoltage(const double voltage) { voltage_ = voltage; }

std::string SAP::GetLogHeader() const {
  std::string str_tmp = "";
  str_tmp += WriteScalar("power_generation" + std::to_string(id_), "W");
  return str_tmp;
}

std::string SAP::GetLogValue() const {
  std::string str_tmp = "";
  str_tmp += WriteScalar(power_generation_);
  return str_tmp;
}

void SAP::MainRoutine(int time_count) {
  libra::Vector<3> sun_dir_b;
  double power_density;

  double time_query = compo_step_time_ * time_count;
  
  if (CsvScenarioInterface::UseCsvSunDirection()) {
    libra::Vector<3> sun_direction_body = CsvScenarioInterface::GetSunDirectionBody(time_query);
    sun_dir_b = libra::normalize(sun_direction_body);
  }
  else{
    libra::Vector<3> sun_pos_b = local_celes_info_->GetPosFromSC_b("SUN");
    sun_dir_b = libra::normalize(sun_pos_b);
  }

  if (CsvScenarioInterface::UseCsvSunFlag()) {
    power_density = srp_->GetSolarConstant() * (int)CsvScenarioInterface::GetSunFlag(time_query);
  }
  else
  {
    power_density = srp_->CalcPowerDensity();
  }

  power_generation_ = cell_efficiency_ * transmission_efficiency_ * power_density * cell_area_ * number_of_parallel_ * number_of_series_ *
                        inner_product(normal_vector_, sun_dir_b);  //仮の実装．実際は太陽方向などからIVカーブを更新．動作電圧に応じた発電電力を求める

  if (power_generation_ < 0) power_generation_ = 0.0;
}
