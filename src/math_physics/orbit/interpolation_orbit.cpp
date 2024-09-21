/**
 * @file interpolation_orbit.cpp
 * @brief Orbit calculation with mathematical interpolation
 */

#include "interpolation_orbit.hpp"

namespace orbit {

InterpolationOrbit::InterpolationOrbit(const size_t degree) {
  std::vector<double> time;
  time.assign(degree, -1.0);
  std::vector<double> position_or_velocity;
  position_or_velocity.assign(degree, 0.0);
  math::Interpolation temp(time, position_or_velocity);
  for (size_t axis = 0; axis < 3; axis++) {
    interpolation_position_or_velocity.push_back(temp);
  }
}

bool InterpolationOrbit::PushAndPopData(const double time, const math::Vector<3> position_or_velocity) {
  bool result;
  for (size_t axis = 0; axis < 3; axis++) {
    result = interpolation_position_or_velocity[axis].PushAndPopData(time, position_or_velocity[axis]);
    if (result == false) {
      return false;
    }
  }
  return true;
}

math::Vector<3> InterpolationOrbit::CalcPositionOrVelocityWithTrigonometric(const double time, const double period) const {
  math::Vector<3> output_position_or_velocity;
  for (size_t axis = 0; axis < 3; axis++) {
    output_position_or_velocity[axis] = interpolation_position_or_velocity[axis].CalcTrigonometric(time, period);
  }
  return output_position_or_velocity;
}

math::Vector<3> InterpolationOrbit::CalcPositionOrVelocityWithPolynomial(const double time) const {
  math::Vector<3> output_position_or_velocity;
  for (size_t axis = 0; axis < 3; axis++) {
    output_position_or_velocity[axis] = interpolation_position_or_velocity[axis].CalcPolynomial(time);
  }
  return output_position_or_velocity;
}

}  // namespace orbit
