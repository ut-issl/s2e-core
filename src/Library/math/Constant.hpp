/**
 * @file
 * @brief header for math constant values
 */

#ifndef CONSTANT_HPP_
#define CONSTANT_HPP_

#include <type_traits>  // std::is_floating_point_v

namespace libra {

// instead of C++20 std::numbers
inline namespace numbers {
template <typename T>
using enable_if_float = std::enable_if_t<std::is_floating_point_v<T>, T>;

#define DEFINE_MATH_CONSTANT(name, value)                  \
  template <typename T>                                    \
  inline constexpr T name##_v = enable_if_float<T>(value); \
  inline constexpr double name = name##_v<double>;

// expanded macro example
// template<typename T>
// inline constexpr T pi_v =
// enable_if_float<T>(3.141592653589793238462643383279502884L); inline constexpr
// double pi = pi_v<double>;

DEFINE_MATH_CONSTANT(pi, 3.141592653589793238462643383279502884L);         /* pi */
DEFINE_MATH_CONSTANT(pi_2, 1.570796326794896619231321691639751442L);       /* pi/2 */
DEFINE_MATH_CONSTANT(pi_4, 0.785398163397448309615660845819875721L);       /* pi/4 */
DEFINE_MATH_CONSTANT(inv_pi, 0.318309886183790671537767526745028724L);     /* 1/pi */
DEFINE_MATH_CONSTANT(inv_sqrtpi, 0.564189583547756286948079451560772586L); /* 1/sqrt(pi) */
DEFINE_MATH_CONSTANT(tau, 6.283185307179586476925286766559005768L);        /* pi*2 */

static_assert(3.14 < pi && pi < 3.15, "pi");

#undef DEFINE_MATH_CONSTANT
}  // namespace numbers

}  // namespace libra

#endif  // CONSTANT_HPP_
