#include "heater.hpp"

#include <cmath>

using namespace std;

Heater::Heater(const int heater_id, const double power_rating, const double lower_threshold, const double upper_threshold)
    : heater_id_(heater_id), power_rating_(power_rating), lower_threshold_(lower_threshold), upper_threshold_(upper_threshold) {
  heater_status_ = HeaterStatus::kOff;
  power_output_ = 0;
}

Heater::~Heater() {}

void Heater::PrintParam(void) {
  cout << "heater_id: " << heater_id_ << endl;
  cout << "  power rating      : " << power_rating_ << endl;
  cout << "  lower threshold   : " << lower_threshold_ << endl;
  cout << "  upper threshold   : " << upper_threshold_ << endl;
  cout << "  status            : " << static_cast<int>(heater_status_) << endl;
  cout << "  power output      : " << power_output_ << endl;
}
