/**
 * @file interpolation_orbit.hpp
 * @brief Orbit calculation with mathematical interpolation
 */

#ifndef S2E_LIBRARY_ORBIT_INTERPOLATION_ORBIT_HPP_
#define S2E_LIBRARY_ORBIT_INTERPOLATION_ORBIT_HPP_

#include <math_physics/math/interpolation.hpp>
#include <math_physics/math/vector.hpp>
#include <vector>

namespace orbit {

/**
 * @class InterpolationOrbit
 * @brief Orbit calculation with mathematical interpolation
 * @note Coordinate and unit of position or velocityis defined by users of this function
 */
class InterpolationOrbit {
 public:
  /**
   * @fn InterpolationOrbit
   * @brief Constructor
   * @param [in] degree: Degree of interpolation
   */
  InterpolationOrbit(const size_t degree);

  /**
   * @fn PushAndPopData
   * @brief Add new data to the tail of the list and erase the head data
   * @param [in] time: time of the new data
   * @param [in] position: Satellite position or velocity of the new data
   */
  bool PushAndPopData(const double time, const math::Vector<3> position_or_velocity);

  /**
   * @fn CalcPositionOrVelocityWithTrigonometric
   * @brief Calculate interpolated position position or velocity with trigonometric method
   * @param [in] time: time
   * @param [in] period: Characteristic period
   * @return Calculated position or velocity
   */
  math::Vector<3> CalcPositionOrVelocityWithTrigonometric(const double time, const double period = 0.0) const;

  /**
   * @fn CalcPositionOrVelocityWithPolynomial
   * @brief Calculate interpolated position or velocity with polynomial method
   * @param [in] time: time
   * @return Calculated position or velocity
   */
  math::Vector<3> CalcPositionOrVelocityWithPolynomial(const double time) const;

  // Getters
  /**
   * @fn GetDegree
   * @return Degree of interpolation
   */
  inline size_t GetDegree() const { return interpolation_position_or_velocity[0].GetDegree(); }
  /**
   * @fn GetTimeList
   * @return Time list registered for the interpolation
   */
  inline std::vector<double> GetTimeList() const { return interpolation_position_or_velocity[0].GetIndependentVariables(); }
  /**
   * @fn GetTimeList
   * @param[in] axis: Axis of position or velocity[0, 2]
   * @return Position or velocity list registered for the interpolation
   * @note return id=2 data when the input axis is over 3.
   */
  inline std::vector<double> GetPositionOrVelocityDataList(const size_t axis) const {
    if (axis > 3) {
      return interpolation_position_or_velocity[2].GetDependentVariables();
    }
    return interpolation_position_or_velocity[axis].GetDependentVariables();
  }

 private:
  std::vector<math::Interpolation> interpolation_position_or_velocity;  // 3D vector of interpolation
};

}  // namespace orbit

#endif  // S2E_LIBRARY_ORBIT_INTERPOLATION_ORBIT_HPP_
