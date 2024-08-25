/**
 * @file surface.cpp
 * @brief Definition of spacecraft surface
 */

#include "surface.hpp"

Surface::Surface(const math::Vector<3> position_b_m, const math::Vector<3> normal_b, const double area_m2, const double reflectivity,
                 const double specularity, const double air_specularity)
    : position_b_m_(position_b_m),
      normal_b_(normal_b),
      area_m2_(area_m2),
      reflectivity_(reflectivity),
      specularity_(specularity),
      air_specularity_(air_specularity) {}
