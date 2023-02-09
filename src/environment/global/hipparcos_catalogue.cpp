/**
 *@file hipparcos_catalogue.cpp
 *@brief Class to calculate star direction with Hipparcos catalogue
 */
#include "hipparcos_catalogue.hpp"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <library/math/constants.hpp>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

HipparcosCatalogue::HipparcosCatalogue(double max_magnitude, string catalogue_path)
    : max_magnitude_(max_magnitude), catalogue_path_(catalogue_path) {}

HipparcosCatalogue::~HipparcosCatalogue() {}

bool HipparcosCatalogue::ReadContents(const string& filename, const char delimiter = ',') {
  if (!IsCalcEnabled) return false;

  ifstream ifs(filename);
  if (!ifs.is_open()) {
    cerr << "file open error(hip_main.csv)";
    return false;
  }

  string title;
  ifs >> title;  // Skip title
  while (!ifs.eof()) {
    HipData hipdata;

    string line;
    ifs >> line;
    replace(line.begin(), line.end(), delimiter, ' ');  // Convert delimiter as space for stringstream
    istringstream streamline(line);

    streamline >> hipdata.hip_num >> hipdata.vmag >> hipdata.ra >> hipdata.de;

    if (hipdata.vmag > max_magnitude_) {
      return true;
    }  // Don't read stars darker than max_magnitude
    hip_catalogue.push_back(hipdata);
  }

  return true;
}

libra::Vector<3> HipparcosCatalogue::GetStarDir_i(int rank) const {
  libra::Vector<3> position;
  double ra = GetRA(rank) * libra::pi / 180;
  double de = GetDE(rank) * libra::pi / 180;

  position[0] = cos(ra) * cos(de);
  position[1] = sin(ra) * cos(de);
  position[2] = sin(de);

  return position;
}

libra::Vector<3> HipparcosCatalogue::GetStarDir_b(int rank, Quaternion q_i2b) const {
  libra::Vector<3> position_i;
  libra::Vector<3> position_b;

  position_i = GetStarDir_i(rank);
  position_b = q_i2b.frame_conv(position_i);

  return position_b;
}

string HipparcosCatalogue::GetLogHeader() const {
  string str_tmp = "";

  return str_tmp;
}

string HipparcosCatalogue::GetLogValue() const {
  string str_tmp = "";

  return str_tmp;
}
