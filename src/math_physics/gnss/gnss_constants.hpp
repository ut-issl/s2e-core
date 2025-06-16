/**
 * @file gnss_constants.hpp
 * @brief header file for GNSS constants
 */

#ifndef S2E_SRC_MATH_PHYSICS_GNSS_GNSS_CONSTANTS_HPP_
#define S2E_SRC_MATH_PHYSICS_GNSS_GNSS_CONSTANTS_HPP_

#include <type_traits>  // std::is_floating_point_v

namespace s2e::math_physics::gnss {

template <typename T>
using enable_if_float = std::enable_if_t<std::is_floating_point_v<T>, T>;

#define DEFINE_GNSS_CONSTANT(name, value)                  \
  template <typename T>                                    \
  inline constexpr T name##_v = enable_if_float<T>(value); \
  inline constexpr double name = name##_v<double>;

inline namespace band_frequencies {
DEFINE_GNSS_CONSTANT(band_frequency_1_Hz, 1575420000.0);  // L1/E1/B1C  frequency [Hz]
DEFINE_GNSS_CONSTANT(band_frequency_2_Hz, 1227600000.0);  // L2         frequency [Hz]
DEFINE_GNSS_CONSTANT(band_frequency_5_Hz, 1176450000.0);  // L5/E5a/B2a frequency [Hz]
}  // namespace band_frequencies
#undef DEFINE_GNSS_CONSTANT
}  // namespace s2e::math_physics::gnss
#endif  // S2E_SRC_MATH_PHYSICS_GNSS_GNSS_CONSTANTS_HPP_