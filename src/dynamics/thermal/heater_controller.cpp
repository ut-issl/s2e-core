#include "heater_controller.hpp"

#include <cassert>
#include <cmath>
using namespace std;

HeaterController::HeaterController(const double lower_threshold_degC, const double upper_threshold_degC)
    : lower_threshold_degC_(lower_threshold_degC), upper_threshold_degC_(upper_threshold_degC) {
  AssertHeaterControllerParams();
}

HeaterController::~HeaterController() {}

void HeaterController::ControlHeater(Heater* p_heater, double temperature_degC) {
  HeaterStatus heater_status = p_heater->GetHeaterStatus();
  if (heater_status == HeaterStatus::kOn) {
    if (temperature_degC > upper_threshold_degC_) {
      p_heater->SetHeaterStatus(HeaterStatus::kOff);
    }
  } else {
    if (temperature_degC < lower_threshold_degC_) {
      p_heater->SetHeaterStatus(HeaterStatus::kOn);
    }
  }
}

void HeaterController::AssertHeaterControllerParams() {
  if (upper_threshold_degC_ < lower_threshold_degC_) {
    std::cerr << "[WARNING] heater_controller: the upper threshold is smaller than the lower threshold. " << std::endl;
    double auto_set_upper_threshold_degC = lower_threshold_degC_ + 10.0;
    std::cerr << "The the upper threshold set as" << auto_set_upper_threshold_degC << std::endl;
    upper_threshold_degC_ = auto_set_upper_threshold_degC;
  }
}

HeaterController InitHeaterController(const std::vector<std::string>& heater_str) {
  using std::stod;

  size_t heater_str_size_defined = 4;                    // Correct size of heater_str
  assert(heater_str.size() == heater_str_size_defined);  // Check if size of heater_str is correct

  // Index to read from heater_str for each parameter
  size_t index_lower_threshold = 2;
  size_t index_upper_threshold = 3;

  double lower_threshold_degC = 0;  // [degC]
  double upper_threshold_degC = 0;  // [degC]

  lower_threshold_degC = stod(heater_str[index_lower_threshold]);
  upper_threshold_degC = stod(heater_str[index_upper_threshold]);

  HeaterController heater_controller(lower_threshold_degC, upper_threshold_degC);
  return heater_controller;
}
