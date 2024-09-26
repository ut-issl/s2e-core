/**
 * @file igs_product_name_handling.hpp
 * @brief Manage IGS product name handling
 * @note IGS product: https://igs.org/products/#orbits_clocks
 *       MGEX product: https://igs.org/mgex/data-products/#orbit_clock
 */

#ifndef S2E_LIBRARY_GNSS_IGS_PRODUCT_NAME_HANDLING_HPP_
#define S2E_LIBRARY_GNSS_IGS_PRODUCT_NAME_HANDLING_HPP_

#include <string>

namespace gnss {

/**
 * @fn GetOrbitClockFileName
 * @brief Return IGS orbit and clock final product file name
 * @param [in] header: Header information (ex. IGS0OPSFIN)
 * @param [in] year_doy: Merged number of year(YYY) and day of year(DDD) (YYYYDDD)
 * @param [in] period: Period of the data (ex. 15M = 15 mins.)
 * @param [in] file_type: File type and extensions (ex. ORB.SP3)
 * @return: file name
 */
std::string GetOrbitClockFinalFileName(const std::string header, const size_t year_doy, const std::string period = "15M",
                                       const std::string file_type = "ORB.SP3") {
  std::string file_name = header + "_" + std::to_string(year_doy) + "0000_01D_" + period + "_" + file_type;

  return file_name;
}

/**
 * @fn MergeYearDoy
 * @brief Return Merged number of year(YYY) and day of year(DDD) (YYYYDDD)
 * @param [in] year: 4-digit Year
 * @param [in] doy: 3-digit Day of year
 * @return: Merged number
 */
size_t MergeYearDoy(const size_t year, const size_t doy) { return year * 1000 + doy; }

/**
 * @fn PerseYearFromYearDoy
 * @brief Return 4-digit year
 * @param [in] year_doy: Merged number of year(YYY) and day of year(DDD) (YYYYDDD)
 * @return: year
 */
size_t PerseYearFromYearDoy(const size_t year_doy) { return year_doy / 1000; }

/**
 * @fn PerseDoyFromYearDoy
 * @brief Return 3-digit day of year
 * @param [in] year_doy: Merged number of year(YYY) and day of year(DDD) (YYYYDDD)
 * @return: day of year
 */
size_t PerseDoyFromYearDoy(const size_t year_doy) {
  size_t year = PerseYearFromYearDoy(year_doy);
  return year_doy - year * 1000;
}

/**
 * @fn IncrementYearDoy
 * @brief Calculate increment value of year_doy considering year update
 * @param [in] year_doy: Merged number of year(YYY) and day of year(DDD) (YYYYDDD)
 * @return: Incremented value
 */
size_t IncrementYearDoy(const size_t year_doy) {
  size_t output = year_doy + 1;
  size_t doy = PerseDoyFromYearDoy(output);

  // Year update
  if (doy > 365) {
    size_t year = PerseYearFromYearDoy(output);
    // Leap year
    if (year % 4 == 0) {
      if (doy > 366) {
        year++;
        doy = 1;
      }
    } else {
      year++;
      doy = 1;
    }
    output = MergeYearDoy(year, doy);
  }

  return output;
}

}  // namespace gnss

#endif  // S2E_LIBRARY_GNSS_IGS_PRODUCT_NAME_HANDLING_HPP_
