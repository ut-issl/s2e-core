#pragma once

#include <Library/math/Vector.hpp>
#include <Library/math/Matrix.hpp>
#include <Library/math/NormalRand.hpp>
#include <Library/math/RandomWalk.hpp>

template<size_t N>
class SensorBase
{
public:
  SensorBase(
    const libra::Matrix<N, N>& scale_factor,
    const libra::Vector<N>& range_to_const_c,
    const libra::Vector<N>& range_to_zero_c,
    const libra::Vector<N>& bias_c,
    const libra::Vector<N>& nr_stddev_c,
    double rw_stepwidth,
    const libra::Vector<N>& rw_stddev_c,
    const libra::Vector<N>& rw_limit_c
  );
  ~SensorBase();

protected:
  libra::Vector<N> Measure(const libra::Vector<N> true_value_c);

private:
  libra::Matrix<N, N>  scale_factor_;  // Scale factor matrix
  libra::Vector<N> range_to_const_c_;
  libra::Vector<N> range_to_zero_c_;
  libra::Vector<N> bias_c_;  // Constant noise vector
  libra::NormalRand nrs_c_[N]; // Normal random
  RandomWalk<N> n_rw_c_;  // Random Walk

  libra::Vector<N> Clip(const libra::Vector<N> input_c);
  void RangeCheck(void);
};

#include "./SensorBase_tfs.hpp" // template function definisions.
