/**
 * @file interpolation.hpp
 * @brief Mathematical interpolation method
 */

#ifndef S2E_LIBRARY_MATH_INTERPOLATION_HPP_
#define S2E_LIBRARY_MATH_INTERPOLATION_HPP_

#include <vector>

namespace libra {

/**
 * @class Interpolation
 * @brief Class for interpolation calculation
 */
class Interpolation {
 public:
  /**
   * @fn Interpolation
   * @brief Constructor without any initialization
   */
  inline Interpolation(std::vector<double> independent_variables, std::vector<double> dependent_variables)
      : independent_variables_(independent_variables), dependent_variables_(dependent_variables) {
    degree_ = independent_variables_.size();
    if (degree_ < 2) {
      // td::cout << "[WARNINGS] Interpolation degree is smaller than 2" << std::endl;
    }
  }

  /**
   * @fn CalcPolynomial
   * @brief Calculate polynomial interpolation
   * @param [in] time_vector: List of given time
   * @return Interpolated value
   */
  double CalcPolynomial(const double independent_variable);

 private:
  std::vector<double> independent_variables_{0.0};
  std::vector<double> dependent_variables_{0.0};
  size_t degree_;

  size_t FindNearestPoint(const double x);
};

}  // namespace libra

#endif  // S2E_LIBRARY_MATH_INTERPOLATION_HPP_
