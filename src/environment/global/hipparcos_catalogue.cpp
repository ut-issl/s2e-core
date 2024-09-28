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

#include "math_physics/math/constants.hpp"
#include "setting_file_reader/initialize_file_access.hpp"

namespace s2e::environment {

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

s2e::math::Vector<3> HipparcosCatalogue::GetStarDirection_i(size_t rank) const {
  s2e::math::Vector<3> direction_i;
  double ra_rad = GetRightAscension_deg(rank) * s2e::math::deg_to_rad;
  double de_rad = GetDeclination_deg(rank) * s2e::math::deg_to_rad;

  direction_i[0] = cos(ra_rad) * cos(de_rad);
  direction_i[1] = sin(ra_rad) * cos(de_rad);
  direction_i[2] = sin(de_rad);

  return direction_i;
}

s2e::math::Vector<3> HipparcosCatalogue::GetStarDirection_b(size_t rank, s2e::math::Quaternion quaternion_i2b) const {
  s2e::math::Vector<3> direction_i;
  s2e::math::Vector<3> direction_b;

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

HipparcosCatalogue* InitHipparcosCatalogue(std::string file_name) {
  setting_file_reader::IniAccess ini_file(file_name);
  const char* section = "HIPPARCOS_CATALOGUE";

  std::string catalogue_path = ini_file.ReadString(section, "catalogue_file_path");
  double max_magnitude = ini_file.ReadDouble(section, "max_magnitude");

  HipparcosCatalogue* hipparcos_catalogue_;
  hipparcos_catalogue_ = new HipparcosCatalogue(max_magnitude, catalogue_path);
  hipparcos_catalogue_->IsCalcEnabled = ini_file.ReadEnable(section, INI_CALC_LABEL);
  hipparcos_catalogue_->is_log_enabled_ = ini_file.ReadEnable(section, INI_LOG_LABEL);
  hipparcos_catalogue_->ReadContents(catalogue_path, ',');

  return hipparcos_catalogue_;
}

} // namespace s2e::environment
