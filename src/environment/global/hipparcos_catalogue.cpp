/**
 *@file hipparcos_catalogue.cpp
 *@brief Class to calculate star direction with Hipparcos catalogue
 */
#include "hipparcos_catalogue.hpp"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "library/math/constants.hpp"

HipparcosCatalogue::HipparcosCatalogue(double max_magnitude, std::string catalogue_path)
    : max_magnitude_(max_magnitude), catalogue_path_(catalogue_path) {}

HipparcosCatalogue::~HipparcosCatalogue() {}

bool HipparcosCatalogue::ReadContents(const std::string& file_name, const char delimiter = ',') {
  if (!IsCalcEnabled) return false;

  std::ifstream ifs(file_name);
  if (!ifs.is_open()) {
    std::cerr << "file open error(hip_main.csv)";
    return false;
  }

  std::string title;
  ifs >> title;  // Skip title
  while (!ifs.eof()) {
    HipparcosData hipparcos_data;

    std::string line;
    ifs >> line;
    std::replace(line.begin(), line.end(), delimiter, ' ');  // Convert delimiter as space for stringstream
    std::istringstream streamline(line);

    streamline >> hipparcos_data.hipparcos_id >> hipparcos_data.visible_magnitude >> hipparcos_data.right_ascension_deg >>
        hipparcos_data.declination_deg;

    if (hipparcos_data.visible_magnitude > max_magnitude_) {
      return true;
    }  // Don't read stars darker than max_magnitude
    hipparcos_catalogue_.push_back(hipparcos_data);
  }

  return true;
}

libra::Vector<3> HipparcosCatalogue::GetStarDirection_i(size_t rank) const {
  libra::Vector<3> direction_i;
  double ra_rad = GetRightAscension_deg(rank) * libra::deg_to_rad;
  double de_rad = GetDeclination_deg(rank) * libra::deg_to_rad;

  direction_i[0] = cos(ra_rad) * cos(de_rad);
  direction_i[1] = sin(ra_rad) * cos(de_rad);
  direction_i[2] = sin(de_rad);

  return direction_i;
}

libra::Vector<3> HipparcosCatalogue::GetStarDirection_b(size_t rank, libra::Quaternion quaternion_i2b) const {
  libra::Vector<3> direction_i;
  libra::Vector<3> direction_b;

  direction_i = GetStarDirection_i(rank);
  direction_b = quaternion_i2b.FrameConversion(direction_i);

  return direction_b;
}

std::string HipparcosCatalogue::GetLogHeader() const {
  std::string str_tmp = "";

  return str_tmp;
}

std::string HipparcosCatalogue::GetLogValue() const {
  std::string str_tmp = "";

  return str_tmp;
}
