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
  inline virtual ~ODE();

  /**
   * @fn RHS
   * @brief Pure virtual function to define the difference equation
   * @param [in] x: Independent variable (e.g. time)
   * @param [in] state: State vector
   * @param [out] rhs: Differentiated value of state vector
   */
  virtual void RHS(double x, const Vector<N>& state, Vector<N>& rhs) = 0;

  /**
   * @fn setup
   * @brief Initialize the state vector
   * @param [in] init_x: Initial value of independent variable
   * @param [in] init_cond: Initial condition of the state vector
   */
  void setup(double init_x, const Vector<N>& init_cond);
  /**
   * @fn setStepWidth
   * @brief Initialize the state vector
   * @param [in] new_step: Initial value of independent variable
   */
  void setStepWidth(double new_step);

  /**
   * @fn step_width
   * @brief Return step width
   */
  inline double step_width() const;

  /**
   * @fn x
   * @brief Return current independent variable
   */
  inline double x() const;

  /**
   * @fn state
   * @brief Return current state vector
   */
  inline const Vector<N>& state() const;

  /**
   * @fn operator []
   * @brief Return element of current state vector
   * @param [in] n: Target element number
   */
  inline double operator[](int n) const;

  /**
   * @fn rhs
   * @brief Return const reference of differentiate state vector
   */
  inline const Vector<N>& rhs() const;

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
  inline libra::Vector<N>& state();

 private:
  double x_;           //!< Latest value of independent variable
  Vector<N> state_;    //!< Latest state vector
  Vector<N> rhs_;      //!< Latest differentiate of the state vector
  double step_width_;  //!< Step width
};

}  // namespace libra

#include "./ordinary_differential_equation_inline_functions.hpp"
#include "./ordinary_differential_equation_template_functions.hpp"

#endif  // S2E_LIBRARY_MATH_ORDINARY_DIFFERENTIA_EQUATION_HPP_
