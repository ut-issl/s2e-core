/**
 * @file residual_magnetic_moment.cpp
 * @brief Definition of RMM (Residual Magnetic Moment)
 */

#include "residual_magnetic_moment.hpp"

namespace s2e::simulation {

ResidualMagneticMoment::ResidualMagneticMoment(const math::Vector<3> constant_value_b_Am2_, const double random_walk_standard_deviation_Am2,
                                               const double random_walk_limit_Am2, const double random_noise_standard_deviation_Am2)
    : constant_value_b_Am2_(constant_value_b_Am2_),
      random_walk_standard_deviation_Am2_(random_walk_standard_deviation_Am2),
      random_walk_limit_Am2_(random_walk_limit_Am2),
      random_noise_standard_deviation_Am2_(random_noise_standard_deviation_Am2) {}

}  // namespace s2e::simulation
