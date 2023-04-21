#include "heater.hpp"

#include <cmath>

using namespace std;

Heater::Heater(const int heater_id, const double power_rating) : heater_id_(heater_id), power_rating_(power_rating) {
  heater_status_ = HeaterStatus::kOff;
  power_output_ = 0;
}

Heater::~Heater() {}

void Heater::SetHeaterStatus(HeaterStatus heater_status) {
  heater_status_ = heater_status;
  power_output_ = static_cast<int>(heater_status_) * power_rating_;
}

void Heater::PrintParam(void) {
  cout << "heater_id: " << heater_id_ << endl;
  cout << "  power rating      : " << power_rating_ << endl;
  cout << "  status            : " << static_cast<int>(heater_status_) << endl;
  cout << "  power output      : " << power_output_ << endl;
}

HeaterController::HeaterController(const double lower_threshold, const double upper_threshold)
    : lower_threshold_(lower_threshold), upper_threshold_(upper_threshold) {}

HeaterController::~HeaterController() {}

void HeaterController::ControlHeater(Heater heater, double temperature_degC) {
  HeaterStatus heater_status = heater.GetHeaterStatus();
  if (heater_status == HeaterStatus::kOn) {
    if (temperature_degC > upper_threshold_) {
      heater.SetHeaterStatus(HeaterStatus::kOff);
    }
  } else {
    if (temperature_degC < lower_threshold_) {
      heater.SetHeaterStatus(HeaterStatus::kOn);
    }
  }
}
