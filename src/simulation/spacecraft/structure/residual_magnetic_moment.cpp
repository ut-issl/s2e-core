/**
 * @file residual_magnetic_moment.cpp
 * @brief Definition of RMM (Residual Magnetic Moment)
 */

#include "residual_magnetic_moment.hpp"

RMMParams::RMMParams(Vector<3> rmm_const_b, double rmm_rwdev, double rmm_rwlimit, double rmm_wnvar)
    : rmm_const_b_(rmm_const_b), rmm_rwdev_(rmm_rwdev), rmm_rwlimit_(rmm_rwlimit), rmm_wnvar_(rmm_wnvar) {}
