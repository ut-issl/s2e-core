/**
 * @file wrapper_nrlmsise00.hpp
 * @brief Functions to wrap NRLMSISE-00 air density model in the Externallibrary
 */

#ifndef S2E_LIBRARY_NRLMSISE00_WRAPPER_NRLMSISE00__HPP_
#define S2E_LIBRARY_NRLMSISE00_WRAPPER_NRLMSISE00__HPP_

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

/**
 * @struct nrlmsise_table
 * @brief Parameters for NRLMSISE calculation
 * @note Ref: https://celestrak.org/SpaceData/SpaceWx-format.php
 */
struct nrlmsise_table {
  int year;          //!< Year
  int month;         //!< Month
  int day;           //!< Day
  double Ap_avg;     //!< Average of Ap-index (Planetary Equivalent Amplitude) for the day
  double F107_adj;   //!< F10.7 (10.7-cm Solar Radio Flux) adjusted to 1AU
  double Ctr81_adj;  //!< Centered 81-day arithmetic average of F10.7 (adjusted)
  double Lst81_adj;  //!< Last 81-day arithmetic average of F10.7 (adjusted).
  double F107_obs;   //!< Observed F10.7
  double Ctr81_obs;  //!< Centered 81-day arithmetic average of F10.7 (observed)
  double Lst81_obs;  //!< Last 81-day arithmetic average of F10.7 (observed).
};

/**
 * @fn CalcNRLMSISE00
 * @brief Read the space weather table file
 * @param [in] decyear: Decimal year of the simulation start time
 * @param [in] latrad: Latitude [rad]
 * @param [in] lonrad: Longitude [rad]
 * @param [in] alt: Altitude [m]
 * @param [in] table: Space Weather table
 * @param [in] is_manual_param: Flag to use manual parameters
 * @param [in] manual_f107: Manual setting F10.7
 * @param [in] manual_f107a: Manual setting averaged F10.7
 * @param [in] manual_ap: Manual setting Ap-index
 * @return Atmospheric density [kg/m3]
 */
double CalcNRLMSISE00(double decyear, double latrad, double lonrad, double alt, const std::vector<nrlmsise_table>& table, bool is_manual_param,
                      double manual_f107, double manual_f107a, double manual_ap);

/**
 * @fn GetSpaceWeatherTable_
 * @brief Read the space weather table file
 * @param [in] decyear: Decimal year of the simulation start time
 * @param [in] endsec: Simulation end time [sec]
 * @param [in] filename: Path to the SpaceWeather file (Ex: ftp://ftp.agi.com/pub/DynamicEarthData/SpaceWeather-v1.2.txt)
 * @param [out] table: Space weather table
 * @return Size of table
 */
int GetSpaceWeatherTable_(double decyear, double endsec, const std::string& filename, std::vector<nrlmsise_table>& table);

/* ------------------------------------------------------------------- */
/* ----------------------- COMPILATION TWEAKS ------------------------ */
/* ------------------------------------------------------------------- */

/* "inlining" of functions */
/*   Some compilers (e.g. gcc) allow the inlining of functions into the
 *   calling routine. This means a lot of overhead can be removed, and
 *   the execution of the program runs much faster. However, the filesize
 *   and thus the loading time is increased.
 */
#ifdef INLINE
#define __inline_double static inline double
#else
#define __inline_double double
#endif

#endif  // S2E_LIBRARY_NRLMSISE00_WRAPPER_NRLMSISE00__HPP_
