/**
 * @file interface_ode.hpp
 * @brief Interface class for ordinary differential equation
 */

#ifndef S2E_LIBRARY_NUMERICAL_INTEGRATION_INTERFACE_ODE_HPP_
#define S2E_LIBRARY_NUMERICAL_INTEGRATION_INTERFACE_ODE_HPP_

#include "../math/vector.hpp"

namespace s2e::numerical_integration {

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
   * @param [in] independent_variable: Independent variable
   * @param [in] state: State vector
   * @return Differentiated value of state vector
   */
  virtual s2e::math::Vector<N> DerivativeFunction(const double independent_variable, const s2e::math::Vector<N>& state) const = 0;
};

}  // namespace s2e::numerical_integration

#endif  // S2E_LIBRARY_NUMERICAL_INTEGRATION_INTERFACE_ODE_HPP_
