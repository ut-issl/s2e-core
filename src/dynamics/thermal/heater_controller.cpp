#include "heater_controller.hpp"

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
  if (upper_threshold_degC_ > lower_threshold_degC_) {
    std::cerr << "[WARNING] heater_controller: the upper threshold is smaller than the lower threshold. " << std::endl;
    double auto_set_upper_threshold_degC = lower_threshold_degC_ + 10.0;
    std::cerr << "The the upper threshold set as" << auto_set_upper_threshold_degC << std::endl;
    upper_threshold_degC_ = auto_set_upper_threshold_degC;
  }
}
