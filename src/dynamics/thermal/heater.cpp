#include "heater.hpp"

#include <cmath>
#include <environment/global/physical_constants.hpp>

using namespace std;

Heater::Heater(const int heater_id, const std::string heater_label, const double power_rating, const double lower_threshold,
               const double upper_threshold)
    : heater_id_(heater_id),
      heater_label_(heater_label),
      power_rating_(power_rating),
      lower_threshold_(lower_threshold),
      upper_threshold_(upper_threshold) {
  status_ = false;
  power_output_ = 0;
}

Heater::~Heater() {}

double Heater::deg2K(double degC) const {
  double temp = degC - environment::absolute_zero_degC;
  return temp;
}

void Heater::SetStatus(bool status) {
  status_ = status;
  if (status_) {
    power_output_ = power_rating_;
  } else {
    power_output_ = 0;
  }
}

void Heater::PrintParam(void) {
  cout << "heater_id: " << heater_id_ << endl;
  cout << "  heater_label      : " << heater_label_ << endl;
  cout << "  power rating      : " << power_rating_ << endl;
  cout << "  lower threshold   : " << lower_threshold_ << endl;
  cout << "  upper threshold   : " << upper_threshold_ << endl;
  cout << "  status            : " << status_ << endl;
  cout << "  power output      : " << power_output_ << endl;
}
