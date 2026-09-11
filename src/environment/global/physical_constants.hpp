/**
 * @file physical_constants.hpp
 * @brief header for physical constant values
 */

#ifndef S2E_ENVIRONMENT_GLOBAL_PHYSICAL_CONSTANT_HPP_
#define S2E_ENVIRONMENT_GLOBAL_PHYSICAL_CONSTANT_HPP_

#include <type_traits>  // std::is_floating_point_v

namespace s2e::environment {

template <typename T>
using enable_if_float = std::enable_if_t<std::is_floating_point_v<T>, T>;

#define DEFINE_PHYSICAL_CONSTANT(name, value)              \
  template <typename T>                                    \
  inline constexpr T name##_v = enable_if_float<T>(value); \
  inline constexpr double name = name##_v<double>;

inline namespace physics {
DEFINE_PHYSICAL_CONSTANT(speed_of_light_m_s, 299792458.0L)                   //!< Speed of light in vacuum [m/s]
DEFINE_PHYSICAL_CONSTANT(boltzmann_constant_J_K, 1.380649e-23L)              //!< Boltzmann constant [J/K]
DEFINE_PHYSICAL_CONSTANT(stefan_boltzmann_constant_W_m2K4, 5.670374419e-8L)  //!< Stefan-Boltzmann constant [W/m2K4]
DEFINE_PHYSICAL_CONSTANT(absolute_zero_degC, -273.15L)                       //!< Absolute zero [degC]
DEFINE_PHYSICAL_CONSTANT(solar_constant_W_m2, 1.366e3L)                      //!< Solar constant [W/m2]
DEFINE_PHYSICAL_CONSTANT(seconds_per_day, 86400.0L)                          //!< Number of seconds in a day [s]
}  // namespace physics

inline namespace astronomy {
// Ref: https://iau-a3.gitlab.io/NSFA/
DEFINE_PHYSICAL_CONSTANT(astronomical_unit_m, 1.49597870700e11L)               //!< Fixed value of the Astronomical unit [m]
DEFINE_PHYSICAL_CONSTANT(gravitational_constant_Nm2_kg2, 6.67428e-11L)         //!< Best estimate of the Gravitational constants [Nm2/kg2]
DEFINE_PHYSICAL_CONSTANT(dt_ut1_utc_s, 32.184L)                                //!< Time difference b/w UT1 and UTC [sec]
DEFINE_PHYSICAL_CONSTANT(julian_date_j2000, 2451545.0L)                        //!< Julian date of J2000 [day]
DEFINE_PHYSICAL_CONSTANT(days_per_julian_century, 36525.0L)                    //!< Conversion constant from Julian century to day [day/century]
DEFINE_PHYSICAL_CONSTANT(earth_equatorial_radius_m, 6378136.6L)                //!< Best estimate of the Earth equatorial radius[m]
DEFINE_PHYSICAL_CONSTANT(earth_polar_radius_m, 6356752.0L)                     //!< The Earth polar radius from NASA's fact sheet [m]
DEFINE_PHYSICAL_CONSTANT(earth_gravitational_constant_m3_s2, 3.986004415e14L)  //!< Best estimate of the Earth's gravitational constants, TT [m3/s2]
DEFINE_PHYSICAL_CONSTANT(earth_mean_angular_velocity_rad_s, 7.292115e-5L)      //!< Best estimate of the Earth's mean angular velocity, TT [rad/s]
DEFINE_PHYSICAL_CONSTANT(earth_flattening, 3.352797e-3L)                       //!< The Earth flattening calculated from the earth radius above
}  // namespace astronomy

#undef DEFINE_PHYSICAL_CONSTANT

inline double degC2K(double degC) { return (degC - absolute_zero_degC); }
inline double K2degC(double K) { return (K + absolute_zero_degC); }

}  // namespace s2e::environment

#endif  // S2E_ENVIRONMENT_GLOBAL_PHYSICAL_CONSTANT_HPP_
