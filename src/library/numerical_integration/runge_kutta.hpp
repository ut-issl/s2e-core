/**
 * @file runge_kutta.hpp
 * @brief Class for General Runge-Kutta method (explicit form)
 * @note Ref: Montenbruck and Gill, Satellite Orbits, 4.1 Runge-Kutta Methods
 */

#ifndef S2E_LIBRARY_NUMERICAL_INTEGRATION_RUNGE_KUTTA_HPP_
#define S2E_LIBRARY_NUMERICAL_INTEGRATION_RUNGE_KUTTA_HPP_

#include <vector>

#include "../math/vector.hpp"
#include "interface_ode.hpp"

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
  inline RungeKutta(const double step_width_s, const InterfaceOde<N>& ode)
      : step_width_s_(step_width_s), ode_(ode), current_time_s_(0.0), current_state_(0.0) {}
  /**
   * @fn ~RungeKutta
   * @brief Destructor
   */
  inline virtual ~RungeKutta(){};

  /**
   * @fn Integrate
   * @brief Update the state vector with the numerical integration
   */
  virtual void Integrate();

  /**
   * @fn GetState
   * @brief Return current state vector
   */
  inline void SetState(const double time_s, const Vector<N>& state) {
    current_state_ = state;
    current_time_s_ = time_s;
  }

  /**
   * @fn GetState
   * @brief Return current state vector
   */
  inline const Vector<N>& GetState() const { return current_state_; }

 protected:
  // Settings
  double step_width_s_;         //!< Step width [s]
  size_t number_of_stages_;     //!< Number of stage for integration (s in the equation)
  size_t approximation_order_;  //!< Order of approximation (p in the equation)

  // Coefficients
  std::vector<double> nodes_;                   //!< Nodes vector for general RK (c vector in the equation)
  std::vector<double> weights_;                 //!< Weights vector for general RK (b vector in the equation)
  std::vector<std::vector<double>> rk_matrix_;  //!< Runge-Kutta matrix for general RK (a matrix in the equation)

  // States
  const InterfaceOde<N>& ode_;  //!< Ordinary differential equation
  double current_time_s_;       //!< Latest value of independent variable
  Vector<N> current_state_;     //!< Latest state vector

  /**
   * @fn CalcSlope
   * @brief Calc slope vector (k in the RK equation)
   */
  std::vector<Vector<N>> CalcSlope();
};

}  // namespace libra

#include "runge_kutta_template.hpp"

#endif  // S2E_LIBRARY_NUMERICAL_INTEGRATION_RUNGE_KUTTA_HPP_
