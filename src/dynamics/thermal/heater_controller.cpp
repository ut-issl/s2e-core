#include "heater_controller.hpp"

#include <cmath>

using namespace std;

HeaterController::HeaterController(const double lower_threshold_degC, const double upper_threshold_degC)
    : lower_threshold_degC_(lower_threshold_degC), upper_threshold_degC_(upper_threshold_degC) {}

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
