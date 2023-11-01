/**
 * @file interpolation_orbit.hpp
 * @brief Orbit calculation with mathematical interpolation
 */

#ifndef S2E_LIBRARY_ORBIT_INTERPOLATION_ORBIT_HPP_
#define S2E_LIBRARY_ORBIT_INTERPOLATION_ORBIT_HPP_

#include <library/math/interpolation.hpp>
#include <library/math/vector.hpp>
#include <vector>

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
  bool PushAndPopData(const double time, const libra::Vector<3> position);

  /**
   * @fn CalcPositionWithTrigonometric
   * @brief Calculate interpolated position with trigonometric method
   * @param [in] time: time
   * @param [in] period: Characteristic period
   * @return Calculated position
   */
  libra::Vector<3> CalcPositionWithTrigonometric(const double time, const double period = 0.0) const;

  // Getter
  inline size_t GetDegree() const { return interpolation_position_[0].GetDegree(); }
  inline std::vector<double> GetTimeList() const { return interpolation_position_[0].GetIndependentVariables(); }
  inline std::vector<double> GetPositionDataList(size_t axis) const { return interpolation_position_[axis].GetDependentVariables(); }
  inline double GetMediumPointTime() const { return this->GetTimeList()[(size_t)(this->GetDegree() / 2)]; }

 private:
  std::vector<libra::Interpolation> interpolation_position_;  // 3D vector of interpolation
};

#endif  // S2E_LIBRARY_ORBIT_INTERPOLATION_ORBIT_HPP_
