/**
 * @file dormand_prince_5.hpp
 * @brief Class for 5th order Dormand and Prince method
 * @note Ref: J. R. Dormand and P. J. Prince, "A family of embedded Runge-Kutta formulae", 1980
 *            J. R. Dormand and P. J. Prince, "Runge-Kutta Triples", 1986
 *            O. Montenbruck and E. Gill, "State interpolation for on-board navigation systems", 2001
 */

#ifndef S2E_LIBRARY_NUMERICAL_INTEGRATION_DORMAND_PRINCE_5_HPP_
#define S2E_LIBRARY_NUMERICAL_INTEGRATION_DORMAND_PRINCE_5_HPP_

#include "embedded_runge_kutta.hpp"

namespace libra::numerical_integration {

/**
 * @class DormandPrince5
 * @brief Class for 5th order Dormand and Prince method
 */
template <size_t N>
class DormandPrince5 : public EmbeddedRungeKutta<N> {
 public:
  /**
   * @fn DormandPrince5
   * @brief Constructor
   * @param [in] step_width: Step width
   * @param [in] ode: Ordinary differential equation
   */
  DormandPrince5(const double step_width, const InterfaceOde<N>& ode);
  /**
   * @fn CalcInterpolationState
   * @brief Calculate interpolation state
   * @param [in] sigma: Sigma value (0 < sigma < 1) for interpolation
   * @return : interpolated state x(t0 + sigma * h)
   */
  Vector<N> CalcInterpolationState(const double sigma) const override;

 private:
  std::vector<libra::Vector<5>> coefficients_;  //!< Coefficients to calculate interpolation weights
  /**
   * @fn CalcInterpolationWeights
   * @brief Calculate weights for interpolation
   * @param [in] sigma: Sigma value (0 < sigma < 1) for interpolation
   * @return : weights for interpolation
   */
  std::vector<double> CalcInterpolationWeights(const double sigma) const;
};

}  // namespace libra::numerical_integration

#include "dormand_prince_5_implementation.hpp"

#endif  // S2E_LIBRARY_NUMERICAL_INTEGRATION_DORMAND_PRINCE_5_HPP_
