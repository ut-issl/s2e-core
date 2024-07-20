/**
 * @file ordinary_differential_equation.hpp
 * @brief Class for Ordinary Differential Equation
 */

#ifndef S2E_LIBRARY_MATH_ORDINARY_DIFFERENTIA_EQUATION_HPP_
#define S2E_LIBRARY_MATH_ORDINARY_DIFFERENTIA_EQUATION_HPP_

#include "./vector.hpp"

namespace math

/**
 * @class OrdinaryDifferentialEquation
 * @brief Class for Ordinary Differential Equation
 */
template <size_t N>
class OrdinaryDifferentialEquation {
 public:
  /**
   * @fn OrdinaryDifferentialEquation
   * @brief Constructor
   * @param [in] step_width_s: Step width
   */
  OrdinaryDifferentialEquation(double step_width_s);
  /**
   * @fn ~OrdinaryDifferentialEquation
   * @brief Destructor
   */
  inline virtual ~OrdinaryDifferentialEquation() {}

  /**
   * @fn DerivativeFunction
   * @brief Pure virtual function to define the difference equation
   * @param [in] independent_variable: Independent variable (e.g. time)
   * @param [in] state: State vector
   * @param [out] derivative: Differentiated value of state vector
   */
  virtual void DerivativeFunction(double independent_variable, const Vector<N>& state, Vector<N>& derivative) = 0;

  /**
   * @fn operator ++
   * @brief Update the state
   */
  OrdinaryDifferentialEquation& operator++();

  /**
   * @fn Update
   * @brief Update the state
   */
  void Update();

  /**
   * @fn Setup
   * @brief Initialize the state vector
   * @param [in] initial_independent_variable: Initial value of independent variable
   * @param [in] initial_state: Initial condition of the state vector
   */
  void Setup(const double initial_independent_variable, const Vector<N>& initial_state);

  /**
   * @fn SetStepWidth
   * @brief Initialize the state vector
   * @param [in] step_width_s: Step width
   */
  inline void SetStepWidth(const double step_width_s) { step_width_s_ = step_width_s; }

  // Getter
  /**
   * @fn GetStepWidth
   * @brief Return step width
   */
  inline double GetStepWidth() const { return step_width_s_; }

  /**
   * @fn GetIndependentVariable
   * @brief Return current independent variable
   */
  inline double GetIndependentVariable() const { return independent_variable_; }

  /**
   * @fn GetState
   * @brief Return current state vector
   */
  inline const Vector<N>& GetState() const { return state_; }

  /**
   * @fn GetDerivative
   * @brief Return const reference of differentiate state vector
   */
  inline const Vector<N>& GetDerivative() const { return derivative_; }

  /**
   * @fn operator []
   * @brief Return element of current state vector
   * @param [in] n: Target element number
   */
  inline double operator[](size_t n) const { return state_[n]; }

 protected:
  /**
   * @fn GetState
   * @brief Return current state vector for inheriting class
   */
  inline libra::Vector<N>& GetState() { return state_; }

 private:
  double independent_variable_;  //!< Latest value of independent variable
  Vector<N> state_;              //!< Latest state vector
  Vector<N> derivative_;         //!< Latest differentiate of the state vector
  double step_width_s_;          //!< Step width
};

}  // namespace libra

#include "./ordinary_differential_equation_template_functions.hpp"

#endif  // S2E_LIBRARY_MATH_ORDINARY_DIFFERENTIA_EQUATION_HPP_
