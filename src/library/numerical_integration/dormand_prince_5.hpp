/**
 * @file dormand_prince_5.hpp
 * @brief Class for 5th order Dormand and Prince method
 */

#ifndef S2E_LIBRARY_NUMERICAL_INTEGRATION_DORMAND_PRINCE_5_HPP_
#define S2E_LIBRARY_NUMERICAL_INTEGRATION_DORMAND_PRINCE_5_HPP_

#include "embedded_runge_kutta.hpp"

namespace libra::numerical_integration {

/**
 * @class DormandPrince5
 * @brief Class for Classical 5th order Dormand and Prince method
 */
template <size_t N>
class DormandPrince5 : public EmbeddedRungeKutta<N> {
 public:
  /**
   * @fn DormandPrince5
   * @brief Constructor
   * @param [in] step_width: Step width
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
  std::vector<libra::Vector<5>> coefficients_;
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
