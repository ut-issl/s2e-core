/**
 * @file surface.cpp
 * @brief Definition of spacecraft surface
 */

#include "surface.hpp"

Surface::Surface(Vector<3> position, Vector<3> normal, double area, double reflectivity, double specularity, double air_specularity)
    : position_b_m_(position),
      normal_b_(normal),
      area_m2_(area),
      reflectivity_(reflectivity),
      specularity_(specularity),
      air_specularity_(air_specularity) {}
