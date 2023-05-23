/**
 * @file runge_kutta.hpp
 * @brief Class for General Runge-Kutta method
 */

#ifndef S2E_LIBRARY_NUMERICAL_INTEGRATION_RUNGE_KUTTA_HPP_
#define S2E_LIBRARY_NUMERICAL_INTEGRATION_RUNGE_KUTTA_HPP_

#include <vector>

#include "../math/vector.hpp"

namespace libra {

/**
 * @class RungeKutta
 * @brief Class for General Runge-Kutta method
 */
template <size_t N>
class RungeKutta {
 public:
  /**
   * @fn RungeKutta
   * @brief Constructor
   * @param [in] step_width_s: Step width [s]
   */
  inline RungeKutta(const double step_width_s) : step_width_s_(step_width_s), current_time_s_(0.0), current_state_(0.0) {}
  /**
   * @fn ~RungeKutta
   * @brief Destructor
   */
  inline virtual ~RungeKutta(){};

  /**
   * @fn Integrate
   * @brief Update the state vector with the numerical integration
   */
  void Integrate();

  /**
   * @fn GetState
   * @brief Return current state vector
   */
  inline const Vector<N>& GetState() const { return current_state_; }

 protected:
  // Settings
  size_t stage_;         //!< Number of stage for integration
  size_t order_;         //!< Number of order for approximation
  double step_width_s_;  //!< Step width [s]

  // Coefficients
  std::vector<Vector<N>> k_;            //!< Slope vector for general RK
  std::vector<double> c_;               //!< Nodes vector for general RK
  std::vector<double> b_;               //!< Weight vector for general RK
  std::vector<std::vector<double>> a_;  //!< Runge-Kutta matrix for general RK

  // States
  double current_time_s_;    //!< Latest value of independent variable
  Vector<N> current_state_;  //!< Latest state vector

  /**
   * @fn DerivativeFunction
   * @brief Pure virtual function to define the difference equation
   * @param [in] time_s: Time as independent variable
   * @param [in] state: State vector
   * @return Differentiated value of state vector
   */
  virtual Vector<N> DerivativeFunction(const double time_s, const Vector<N>& state) = 0;

  /**
   * @fn CalcSlope
   * @brief Calc slope
   */
  void CalcSlope();
};

}  // namespace libra

#include "runge_kutta_template.hpp"

#endif  // S2E_LIBRARY_NUMERICAL_INTEGRATION_RUNGE_KUTTA_HPP_
