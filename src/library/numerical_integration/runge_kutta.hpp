/**
 * @file runge_kutta.hpp
 * @brief Base Class for General Runge-Kutta method (explicit form)
 * @note Ref: Montenbruck and Gill, Satellite Orbits, 4.1 Runge-Kutta Methods
 */

#ifndef S2E_LIBRARY_NUMERICAL_INTEGRATION_RUNGE_KUTTA_HPP_
#define S2E_LIBRARY_NUMERICAL_INTEGRATION_RUNGE_KUTTA_HPP_

#include "numerical_integrator.hpp"

namespace libra::numerical_integrator {

/**
 * @class RungeKutta
 * @brief Base Class for General Runge-Kutta method
 */
template <size_t N>
class RungeKutta : public NumericalIntegrator<N> {
 public:
  /**
   * @fn RungeKutta
   * @brief Constructor
   * @param [in] step_width_s: Step width [s]
   * @param [in] ode: Ordinary differential equation
   */
  inline RungeKutta(const double step_width_s, const InterfaceOde<N>& ode) : NumericalIntegrator<N>(step_width_s, ode) {}
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

 protected:
  // Settings
  size_t number_of_stages_;     //!< Number of stage for integration (s in the equation)
  size_t approximation_order_;  //!< Order of approximation (p in the equation)

  // Coefficients should be defined by child class
  std::vector<double> nodes_;                   //!< Nodes vector for general RK (c vector in the equation)
  std::vector<double> weights_;                 //!< Weights vector for general RK (b vector in the equation)
  std::vector<std::vector<double>> rk_matrix_;  //!< Runge-Kutta matrix for general RK (a matrix in the equation)
  std::vector<Vector<N>> slope_;                //!< Slope vector for general RK (k vector in the equation)

  /**
   * @fn CalcSlope
   * @brief Calc slope vector (k in the RK equation)
   */
  void CalcSlope();
};

}  // namespace libra::numerical_integrator

#include "runge_kutta_template.hpp"

#endif  // S2E_LIBRARY_NUMERICAL_INTEGRATION_RUNGE_KUTTA_HPP_
