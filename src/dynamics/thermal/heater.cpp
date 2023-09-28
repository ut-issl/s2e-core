#include "heater.hpp"

#include <cmath>

using namespace std;

Heater::Heater(const unsigned int heater_id, const double power_rating_W) : heater_id_(heater_id), power_rating_W_(power_rating_W) {
  AssertHeaterParams();
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

void Heater::AssertHeaterParams(void) {
  // Heater ID must be larger than 1
  if (heater_id_ < 1) {
    std::cerr << "[WARNING] heater: heater ID is smaller than 1. " << std::endl;
    heater_id_ += 1;
    std::cerr << "The heater ID is set as " << heater_id_ << std::endl;
  }
  // Power rating must be larger than 0
  if (power_rating_W_ < 0.0) {
    std::cerr << "[WARNING] heater: power rating is smaller than 0.0 [W]. " << std::endl;
    std::cerr << "The value is set as 0.0." << std::endl;
    power_rating_W_ = 0.0;
  }
}
