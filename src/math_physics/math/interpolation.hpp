/**
 * @file interpolation.hpp
 * @brief Mathematical interpolation method
 */

#ifndef S2E_LIBRARY_MATH_INTERPOLATION_HPP_
#define S2E_LIBRARY_MATH_INTERPOLATION_HPP_

#include <iostream>
#include <vector>

namespace s2e::math {

/**
 * @class Interpolation
 * @brief Class for interpolation calculation
 */
class Interpolation {
 public:
  /**
   * @fn Interpolation
   * @brief Constructor without any initialization
   * @note The size of independent variables automatically set as degree of interpolation
   * @param[in] independent_variables: Set of independent variables
   * @param[in] dependent_variables: Set of independent variables
   */
  inline Interpolation(std::vector<double> independent_variables, std::vector<double> dependent_variables)
      : independent_variables_(independent_variables), dependent_variables_(dependent_variables) {
    degree_ = independent_variables_.size();
    if (degree_ < 2) {
      std::cout << "[WARNINGS] Interpolation degree is smaller than 2" << std::endl;
    }
  }

  /**
   * @fn CalcPolynomial
   * @brief Calculate polynomial interpolation with Neville's algorithm
   * @note Ref: Numerical Recipes in C, Section. 3.1
   * @param [in] x: Target independent variable
   * @return Interpolated value at x
   */
  double CalcPolynomial(const double x) const;

  /**
   * @fn CalcTrigonometric
   * @brief Calculate trigonometric interpolation
   * @param [in] x: Target independent variable
   * @param [in] period: Characteristic period
   * @return Interpolated value at x
   */
  double CalcTrigonometric(const double x, const double period = 1.0) const;

  /**
   * @fn PushAndPopData
   * @brief Push new data to the tail and erase the head data
   * @param [in] independent_variable: Independent variable of data
   * @param [in] dependent_variable: Dependent variable of data
   * @return true: success, false: the independent variable of new data is smaller than other data
   */
  bool PushAndPopData(const double independent_variable, const double dependent_variable);

  // Getters
  /**
   * @fn GetDegree
   * @return Degree of interpolation
   */
  inline size_t GetDegree() const { return degree_; }
  /**
   * @fn GetIndependentVariables
   * @return List of independent variables
   */
  inline std::vector<double> GetIndependentVariables() const { return independent_variables_; }
  /**
   * @fn GetDependentVariables
   * @return List of dependent variables
   */
  inline std::vector<double> GetDependentVariables() const { return dependent_variables_; }

 private:
  std::vector<double> independent_variables_{0.0};  //!< List of independent variable
  std::vector<double> dependent_variables_{0.0};    //!< List of dependent variable
  size_t degree_;                                   //!< Degree of interpolation

  /**
   * @fn FindNearestPoint
   * @brief Find nearest independent variables index
   * @param [in] x: Target independent variable
   * @return Index of the nearest independent variables
   */
  size_t FindNearestPoint(const double x) const;
};

}  // namespace s2e::math

#endif  // S2E_LIBRARY_MATH_INTERPOLATION_HPP_
