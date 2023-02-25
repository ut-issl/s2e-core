/**
 * @file ordinary_differential_equation.hpp
 * @brief Class for Ordinary Difference Equation
 */

#ifndef S2E_LIBRARY_MATH_ORDINARY_DIFFERENTIA_EQUATION_HPP_
#define S2E_LIBRARY_MATH_ORDINARY_DIFFERENTIA_EQUATION_HPP_

#include "./vector.hpp"

namespace libra {

/**
 * @class ODE
 * @brief Class for Ordinary Difference Equation
 */
template <size_t N>
class ODE {
 public:
  /**
   * @fn ODE
   * @brief Constructor
   * @param [in] step_width: Step width
   */
  ODE(double step_width);
  /**
   * @fn ~ODE
   * @brief Destructor
   */
  inline virtual ~ODE() {}

  /**
   * @fn RHS
   * @brief Pure virtual function to define the difference equation
   * @param [in] x: Independent variable (e.g. time)
   * @param [in] state: State vector
   * @param [out] derivative: Differentiated value of state vector
   */
  virtual void RHS(double x, const Vector<N>& state, Vector<N>& derivative) = 0;

  /**
   * @fn setup
   * @brief Initialize the state vector
   * @param [in] initial_independent_variable: Initial value of independent variable
   * @param [in] initial_state: Initial condition of the state vector
   */
  void setup(double initial_independent_variable, const Vector<N>& initial_state);
  /**
   * @fn setStepWidth
   * @brief Initialize the state vector
   * @param [in] step_width: Step width
   */
  void setStepWidth(double step_width);

  /**
   * @fn step_width
   * @brief Return step width
   */
  inline double step_width() const { return step_width_; }

  /**
   * @fn x
   * @brief Return current independent variable
   */
  inline double x() const { return independent_variable_; }

  /**
   * @fn state
   * @brief Return current state vector
   */
  inline const Vector<N>& state() const { return state_; }

  /**
   * @fn operator []
   * @brief Return element of current state vector
   * @param [in] n: Target element number
   */
  inline double operator[](int n) const { return state_[n]; }

  /**
   * @fn rhs
   * @brief Return const reference of differentiate state vector
   */
  inline const Vector<N>& rhs() const { return derivative_; }

  /**
   * @fn operator ++
   * @brief Update the state
   */
  ODE& operator++();

  /**
   * @fn Update
   * @brief Update the state
   */
  void Update();

 protected:
  /**
   * @fn state
   * @brief Return current state vector for inheriting class
   */
  inline libra::Vector<N>& state() { return state_; }

 private:
  double independent_variable_;  //!< Latest value of independent variable
  Vector<N> state_;              //!< Latest state vector
  Vector<N> derivative_;         //!< Latest differentiate of the state vector
  double step_width_;            //!< Step width
};

}  // namespace libra

#include "./ordinary_differential_equation_template_functions.hpp"

#endif  // S2E_LIBRARY_MATH_ORDINARY_DIFFERENTIA_EQUATION_HPP_
