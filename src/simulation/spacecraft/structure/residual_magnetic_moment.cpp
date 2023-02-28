/**
 * @file residual_magnetic_moment.cpp
 * @brief Definition of RMM (Residual Magnetic Moment)
 */

#include "residual_magnetic_moment.hpp"

RMMParams::RMMParams(Vector<3> rmm_const_b, double rmm_rwdev, double rmm_rwlimit, double rmm_wnvar)
    : constant_value_b_Am2_(rmm_const_b),
      random_walk_standard_deviation_Am2(rmm_rwdev),
      random_walk_limit_Am2(rmm_rwlimit),
      random_noise_standard_deviation_Am2(rmm_wnvar) {}
