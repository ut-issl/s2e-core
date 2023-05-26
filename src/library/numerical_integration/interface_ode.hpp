/**
 * @file interface_ode.hpp
 * @brief Interface class for ordinary differential equation
 */

#ifndef S2E_LIBRARY_NUMERICAL_INTEGRATION_INTERFACE_ODE_HPP_
#define S2E_LIBRARY_NUMERICAL_INTEGRATION_INTERFACE_ODE_HPP_

#include <cstddef>

namespace libra {

/**
 * @class InterfaceOde
 * @brief Interface class for ordinary differential equation
 */
template <size_t N>
class InterfaceOde {
 public:
  /**
   * @fn DerivativeFunction
   * @brief Pure virtual function to define the difference equation
   * @param [in] time_s: Time as independent variable
   * @param [in] state: State vector
   * @return Differentiated value of state vector
   */
  virtual Vector<N> DerivativeFunction(const double time_s, const Vector<N>& state) const = 0;
};

}  // namespace libra

#endif  // S2E_LIBRARY_NUMERICAL_INTEGRATION_INTERFACE_ODE_HPP_
