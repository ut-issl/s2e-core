/**
 * @file gnss_satellite_number.hpp
 * @brief Manage satellite number defined in RINEX v4
 */

#ifndef S2E_LIBRARY_GNSS_GNSS_SATELLITE_NUMBER_HPP_
#define S2E_LIBRARY_GNSS_GNSS_SATELLITE_NUMBER_HPP_

#include <stdint.h>

#include <string>

namespace gnss {

// GNSS satellite number definition
// TODO: Move to initialized file?
const size_t kNumberOfGpsSatellite = 32;      //!< Number of GPS satellites
const size_t kNumberOfGlonassSatellite = 26;  //!< Number of GLONASS satellites
const size_t kNumberOfGalileoSatellite = 28;  //!< Number of Galileo satellites
const size_t kNumberOfBeidouSatellite = 62;   //!< Number of BeiDou satellites
const size_t kNumberOfQzssSatellite = 5;      //!< Number of QZSS satellites
const size_t kNumberOfNavicSatellite = 7;     //!< Number of NavIC satellites

const size_t kTotalNumberOfGnssSatellite = kNumberOfGpsSatellite + kNumberOfGlonassSatellite + kNumberOfGalileoSatellite + kNumberOfBeidouSatellite +
                                           kNumberOfQzssSatellite + kNumberOfNavicSatellite;  //<! Total number of GNSS satellites

// GNSS satellite index definitions
const size_t kGpsIndexBegin = 0;                                                   //!< Begin value of index for GPS satellites
const size_t kGlonassIndexBegin = kGpsIndexBegin + kNumberOfGpsSatellite;          //!< Begin value of index for GLONASS satellites
const size_t kGalileoIndexBegin = kGlonassIndexBegin + kNumberOfGlonassSatellite;  //!< Begin value of index for Galileo satellites
const size_t kBeidouIndexBegin = kGalileoIndexBegin + kNumberOfGalileoSatellite;   //!< Begin value of index for BeiDou satellites
const size_t kQzssIndexBegin = kBeidouIndexBegin + kNumberOfBeidouSatellite;       //!< Begin value of index for QZSS satellites
const size_t kNavicIndexBegin = kQzssIndexBegin + kNumberOfQzssSatellite;          //!< Begin value of index for NavIC satellites

/**
 * @fn ConvertGnssSatelliteNumberToIndex
 * @brief Calculate index of GNSS satellite defined in S2E from GNSS satellite number defined in RINEX v4
 * @return Index of GNSS satellite defined in this class. or INT32_MAX when the input is wrong.
 */
size_t ConvertGnssSatelliteNumberToIndex(const std::string satellite_number);

/**
 * @fn ConvertIndexToGnssSatelliteNumber
 * @brief Calculate GNSS satellite number defined in RINEX v4 from index of GNSS satellite defined in this class
 * @return GNSS satellite number defined in RINEX v4. or err when the input is wrong.
 */
std::string ConvertIndexToGnssSatelliteNumber(const size_t index);

}  // namespace gnss

#endif  // S2E_LIBRARY_GNSS_GNSS_SATELLITE_NUMBER_HPP_
