#include "heater.hpp"

#include <cmath>
#include <environment/global/physical_constants.hpp>

using namespace std;

Heater::Heater(const int heater_id, const std::string heater_label, const double power_rating, const double lower_threshold, const double upper_threshold)
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

int Heater::GetHeaterId(void) const { return heater_id_; }

std::string Heater::GetHeaterLabel(void) const { return std::string(); }

double Heater::GetPowerRating(void) const { return power_rating_; }

double Heater::GetLowerThreshold_deg(void) const { return lower_threshold_; }

double Heater::GetUpperThreshold_deg(void) const { return upper_threshold_; }

double Heater::GetLowerThreshold_K(void) const { return deg2K(lower_threshold_); }

double Heater::GetUpperThreshold_K(void) const { return deg2K(upper_threshold_); }

double Heater::GetStatus(void) const { return status_; }

double Heater::GetPowerOutput(void) const { return power_output_; }

void Heater::SetLowerThreshold(double lower_threshold_deg) { lower_threshold_ = lower_threshold_deg; }

void Heater::SetUpperThreshold(double upper_threshold_deg) { upper_threshold_ = upper_threshold_deg; }

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