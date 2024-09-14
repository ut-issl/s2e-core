/**
 * @file interpolation_orbit.hpp
 * @brief Orbit calculation with mathematical interpolation
 */

#ifndef S2E_LIBRARY_ORBIT_INTERPOLATION_ORBIT_HPP_
#define S2E_LIBRARY_ORBIT_INTERPOLATION_ORBIT_HPP_

#include <math_physics/math/interpolation.hpp>
#include <math_physics/math/vector.hpp>
#include <vector>

namespace s2e::orbit {

/**
 * @class InterpolationOrbit
 * @brief Orbit calculation with mathematical interpolation
 * @note Coordinate and unit of position is defined by users of this function
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
   * @param [in] position: Satellite position of the new data
   */
  bool PushAndPopData(const double time, const s2e::math::Vector<3> position);

  /**
   * @fn CalcPositionWithTrigonometric
   * @brief Calculate interpolated position with trigonometric method
   * @param [in] time: time
   * @param [in] period: Characteristic period
   * @return Calculated position
   */
  s2e::math::Vector<3> CalcPositionWithTrigonometric(const double time, const double period = 0.0) const;

  // Getters
  /**
   * @fn GetDegree
   * @return Degree of interpolation
   */
  inline size_t GetDegree() const { return interpolation_position_[0].GetDegree(); }
  /**
   * @fn GetTimeList
   * @return Time list registered for the interpolation
   */
  inline std::vector<double> GetTimeList() const { return interpolation_position_[0].GetIndependentVariables(); }
  /**
   * @fn GetTimeList
   * @param[in] axis: Axis of position [0, 2]
   * @return Position list registered for the interpolation
   * @note return id=2 data when the input axis is over 3.
   */
  inline std::vector<double> GetPositionDataList(const size_t axis) const {
    if (axis > 3) {
      return interpolation_position_[2].GetDependentVariables();
    }
    return interpolation_position_[axis].GetDependentVariables();
  }

 private:
  std::vector<s2e::math::Interpolation> interpolation_position_;  // 3D vector of interpolation
};

}  // namespace s2e::orbit

#endif  // S2E_LIBRARY_ORBIT_INTERPOLATION_ORBIT_HPP_
