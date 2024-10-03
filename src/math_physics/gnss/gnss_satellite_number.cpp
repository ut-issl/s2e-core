/**
 * @file gnss_satellite_number.cpp
 * @brief Manage satellite number defined in RINEX v4
 */

#include "gnss_satellite_number.hpp"

namespace s2e::gnss {

size_t ConvertGnssSatelliteNumberToIndex(const std::string satellite_number) {
  switch (satellite_number.front()) {
    case 'G':
      return stoi(satellite_number.substr(1)) + kGpsIndexBegin - 1;
    case 'R':
      return stoi(satellite_number.substr(1)) + kGlonassIndexBegin - 1;
    case 'E':
      return stoi(satellite_number.substr(1)) + kGalileoIndexBegin - 1;
    case 'C':
      return stoi(satellite_number.substr(1)) + kBeidouIndexBegin - 1;
    case 'J':
      return stoi(satellite_number.substr(1)) + kQzssIndexBegin - 1;
    case 'I':
      return stoi(satellite_number.substr(1)) + kNavicIndexBegin - 1;
    default:
      return UINT32_MAX;
      break;
  }
}

std::string ConvertIndexToGnssSatelliteNumber(const size_t index) {
  std::string output;
  size_t prn_number = 0;

  if (index < kGlonassIndexBegin) {
    output = 'G';
    prn_number = index - kGpsIndexBegin + 1;
  } else if (index < kGalileoIndexBegin) {
    output = 'R';
    prn_number = index - kGlonassIndexBegin + 1;
  } else if (index < kBeidouIndexBegin) {
    output = 'E';
    prn_number = index - kGalileoIndexBegin + 1;
  } else if (index < kQzssIndexBegin) {
    output = 'C';
    prn_number = index - kBeidouIndexBegin + 1;
  } else if (index < kNavicIndexBegin) {
    output = 'J';
    prn_number = index - kQzssIndexBegin + 1;
  } else if (index < kTotalNumberOfGnssSatellite) {
    output = 'I';
    prn_number = index - kNavicIndexBegin + 1;
  } else {
    return "err";
  }

  if (prn_number < 10) {
    output += '0';
  }
  output += std::to_string(prn_number);

  return output;
}

}  // namespace s2e::gnss
