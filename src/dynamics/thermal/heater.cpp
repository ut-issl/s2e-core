#include "heater.hpp"

#include <cmath>

using namespace std;

Heater::Heater(const int heater_id, const double power_rating_W) : heater_id_(heater_id), power_rating_W_(power_rating_W) {
  heater_status_ = HeaterStatus::kOff;
  power_output_W_ = 0;
}

Heater::~Heater() {}

void Heater::SetHeaterStatus(HeaterStatus heater_status) {
  heater_status_ = heater_status;
  if (heater_status_ == HeaterStatus::kOn) {
    power_output_W_ = power_rating_W_;
  } else if (heater_status_ == HeaterStatus::kOff) {
    power_output_W_ = 0;
  }
}

void Heater::PrintParam(void) {
  cout << "heater_id: " << heater_id_ << endl;
  cout << "  power rating      : " << power_rating_W_ << endl;
  cout << "  status            : " << static_cast<int>(heater_status_) << endl;
  cout << "  power output      : " << power_output_W_ << endl;
}

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
