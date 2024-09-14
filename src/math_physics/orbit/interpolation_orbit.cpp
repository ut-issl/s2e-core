/**
 * @file interpolation_orbit.cpp
 * @brief Orbit calculation with mathematical interpolation
 */

#include "interpolation_orbit.hpp"

namespace orbit {

InterpolationOrbit::InterpolationOrbit(const size_t degree) {
  std::vector<double> time;
  time.assign(degree, -1.0);
  std::vector<double> position;
  position.assign(degree, 0.0);
  s2e::math::Interpolation temp(time, position);
  for (size_t axis = 0; axis < 3; axis++) {
    interpolation_position_.push_back(temp);
  }
}

bool InterpolationOrbit::PushAndPopData(const double time, const s2e::math::Vector<3> position) {
  bool result;
  for (size_t axis = 0; axis < 3; axis++) {
    result = interpolation_position_[axis].PushAndPopData(time, position[axis]);
    if (result == false) {
      return false;
    }
  }
  return true;
}

s2e::math::Vector<3> InterpolationOrbit::CalcPositionWithTrigonometric(const double time, const double period) const {
  s2e::math::Vector<3> output_position;
  for (size_t axis = 0; axis < 3; axis++) {
    output_position[axis] = interpolation_position_[axis].CalcTrigonometric(time, period);
  }
  return output_position;
}

}  // namespace orbit
