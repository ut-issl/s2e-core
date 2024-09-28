#include "heater.hpp"

#include <cassert>
#include <cmath>

using namespace std;

namespace s2e::dynamics::thermal {

Heater::Heater(const size_t heater_id, const double power_rating_W) : heater_id_(heater_id), power_rating_W_(power_rating_W) {
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

/* Import heater properties by reading CSV File (heaters.csv)

[File Formats of heater.csv]
column 1: Heater_id(int, Use values larger than or equal to 1)
column 2: Power Rating (double, [W])
column 3: Lower threshold of control (double, [degC])
column 4: Upper threshold of control (double, [degC])

First row is for Header, data begins from the second row
*/

Heater InitHeater(const std::vector<std::string>& heater_str) {
  using std::stod;
  using std::stoi;

  size_t heater_str_size_defined = 4;                    // Correct size of heater_str
  assert(heater_str.size() == heater_str_size_defined);  // Check if size of heater_str is correct

  size_t heater_id = 0;
  double power_rating_W = 0;  // [W]

  // Index to read from heater_str for each parameter
  size_t index_heater_id = 0;
  size_t index_power_rating = 1;

  heater_id = stoi(heater_str[index_heater_id]);
  power_rating_W = stod(heater_str[index_power_rating]);

  Heater heater(heater_id, power_rating_W);
  return heater;
}

}  // namespace s2e::dynamics::thermal
