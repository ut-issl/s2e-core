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

bool HipparcosCatalogue::ReadContents(const std::string& filename, const char delimiter = ',') {
  if (!IsCalcEnabled) return false;

  std::ifstream ifs(filename);
  if (!ifs.is_open()) {
    std::cerr << "file open error(hip_main.csv)";
    return false;
  }

  std::string title;
  ifs >> title;  // Skip title
  while (!ifs.eof()) {
    HipData hipdata;

    std::string line;
    ifs >> line;
    std::replace(line.begin(), line.end(), delimiter, ' ');  // Convert delimiter as space for stringstream
    std::istringstream streamline(line);

    streamline >> hipdata.hip_num >> hipdata.vmag >> hipdata.ra >> hipdata.de;

    if (hipdata.vmag > max_magnitude_) {
      return true;
    }  // Don't read stars darker than max_magnitude
    hipparcos_catalogue_.push_back(hipdata);
  }

  return true;
}

libra::Vector<3> HipparcosCatalogue::GetStarDirection_i(int rank) const {
  libra::Vector<3> position;
  // TODO: Check unit of ra and de
  double ra = GetRA(rank) * libra::pi / 180;
  double de = GetDE(rank) * libra::pi / 180;

  position[0] = cos(ra) * cos(de);
  position[1] = sin(ra) * cos(de);
  position[2] = sin(de);

  return position;
}

libra::Vector<3> HipparcosCatalogue::GetStarDirection_b(int rank, Quaternion q_i2b) const {
  libra::Vector<3> position_i;
  libra::Vector<3> position_b;

  position_i = GetStarDirection_i(rank);
  position_b = q_i2b.frame_conv(position_i);

  return position_b;
}

std::string HipparcosCatalogue::GetLogHeader() const {
  std::string str_tmp = "";

  return str_tmp;
}

std::string HipparcosCatalogue::GetLogValue() const {
  std::string str_tmp = "";

  return str_tmp;
}
