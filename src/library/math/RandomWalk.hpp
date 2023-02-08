/**
 * @file RandomWalk.hpp
 * @brief Class to calculate random wark value
 */

#pragma once

#include "./normal_randomization.hpp"
#include "./ordinary_differential_equation.hpp"
#include "./Vector.hpp"

/**
 * @class RandomWalk
 * @brief Class to calculate random wark value
 */
template <size_t N>
class RandomWalk : public libra::ODE<N> {
 public:
  /**
   * @fn RandomWalk
   * @brief Constructor
   * @param step_width: Step width
   * @param stddev: Standard deviation of random walk excitation noise
   * @param limit: Limit of random walk
   */
  RandomWalk(double step_width, const libra::Vector<N>& stddev, const libra::Vector<N>& limit);

  /**
   * @fn RHS
   * @brief Override function of ODE to define the difference equation
   * @param [in] x: Independent variable (e.g. time)
   * @param [in] state: State vector
   * @param [out] rhs: Differentiated value of state vector
   */
  virtual void RHS(double x, const libra::Vector<N>& state, libra::Vector<N>& rhs);

 private:
  libra::Vector<N> limit_;    //!< Limit of random walk
  libra::NormalRand nrs_[N];  //!< Random walk excitation noise
};

#include "./RandomWalk_tfs.hpp"  // template function definisions.
