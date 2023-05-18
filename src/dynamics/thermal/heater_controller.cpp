#include "heater_controller.hpp"

#include <cmath>

using namespace std;

HeaterController::HeaterController(const double lower_threshold_degC, const double upper_threshold_degC)
    : lower_threshold_degC_(lower_threshold_degC), upper_threshold_degC_(upper_threshold_degC) {}

HeaterController::~HeaterController() {}

void HeaterController::ControlHeater(Heater heater, double temperature_degC) {
  HeaterStatus heater_status = heater.GetHeaterStatus();
  if (heater_status == HeaterStatus::kOn) {
    if (temperature_degC > upper_threshold_degC_) {
      heater.SetHeaterStatus(HeaterStatus::kOff);
    }
  } else {
    if (temperature_degC < lower_threshold_degC_) {
      heater.SetHeaterStatus(HeaterStatus::kOn);
    }
  }
}
