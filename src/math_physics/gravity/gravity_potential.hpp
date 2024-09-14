/**
 * @file gravity_potential.hpp
 * @brief Class to calculate gravity potential
 */

#ifndef S2E_LIBRARY_GRAVITY_GRAVITY_POTENTIAL_HPP_
#define S2E_LIBRARY_GRAVITY_GRAVITY_POTENTIAL_HPP_

#include <environment/global/physical_constants.hpp>
#include <vector>

#include "../math/matrix.hpp"
#include "../math/vector.hpp"

namespace gravity {

/**
 * @class GravityPotential
 * @brief Class to calculate gravity potential
 */
class GravityPotential {
 public:
  /**
   * @fn GravityPotential
   * @brief Constructor
   */
  GravityPotential(const double gravity_constants_m3_s2 = environment::earth_gravitational_constant_m3_s2,
                   const double center_body_radius_m = environment::earth_equatorial_radius_m)
      : gravity_constants_m3_s2_(gravity_constants_m3_s2), center_body_radius_m_(center_body_radius_m) {}
  /**
   * @fn GravityPotential
   * @brief Constructor
   * @param [in] degree: Maximum degree setting to calculate the geo-potential
   */
  GravityPotential(const size_t degree, const std::vector<std::vector<double>> cosine_coefficients,
                   const std::vector<std::vector<double>> sine_coefficients,
                   const double gravity_constants_m3_s2 = environment::earth_gravitational_constant_m3_s2,
                   const double center_body_radius_m = environment::earth_equatorial_radius_m);
  /**
   * @fn ~GravityPotential
   * @brief Destructor
   */
  ~GravityPotential() {}

  /**
   * @fn CalcAcceleration_xcxf_m_s2
   * @brief Calculate the high-order earth gravity in the XCXF frame (Arbitrary celestial body centered and fixed frame)
   * @param [in] position_xcxf_m: Position of the spacecraft in the XCXF frame [m]
   * @return Acceleration in XCXF frame [m/s2]
   */
  s2e::math::Vector<3> CalcAcceleration_xcxf_m_s2(const s2e::math::Vector<3> &position_xcxf_m);

  /**
   * @fn CalcAcceleration_xcxf_m_s2
   * @brief Calculate the high-order earth gravity in the XCXF frame (Arbitrary celestial body centered and fixed frame)
   * @param [in] position_xcxf_m: Position of the spacecraft in the XCXF frame [m]
   * @return Partial derivative of acceleration in XCXF frame [-/s2]
   */
  s2e::math::Matrix<3, 3> CalcPartialDerivative_xcxf_s2(const s2e::math::Vector<3> &position_xcxf_m);

 private:
  size_t degree_ = 0;                   //!< Maximum degree
  size_t n_ = 0, m_ = 0;                //!< Degree and order (FIXME: follow naming rule)
  std::vector<std::vector<double>> c_;  //!< Cosine coefficients
  std::vector<std::vector<double>> s_;  //!< Sine coefficients
  double gravity_constants_m3_s2_;      //!< Gravity constant of the center body [m3/s2]
  double center_body_radius_m_;         //!< Radius of the center body [m]

  // calculation
  double radius_m_ = 0.0;                                    //!< Radius [m]
  double xcxf_x_m_ = 0.0, xcxf_y_m_ = 0.0, xcxf_z_m_ = 0.0;  //!< Spacecraft position in XCXF frame [m]

  /**
   * @fn v_w_nn_update
   * @brief Calculate V and W function for n = m
   * @note FIXME: fix function name
   */
  void v_w_nn_update(double *v_nn, double *w_nn, const double v_prev, const double w_prev);

  /**
   * @fn v_w_nm_update
   * @brief Calculate V and W function for n not equal m
   * @note FIXME: fix function name
   */
  void v_w_nm_update(double *v_nm, double *w_nm, const double v_prev, const double w_prev, const double v_prev2, const double w_prev2);
};

}  // namespace gravity

#endif  // S2E_LIBRARY_GRAVITY_GRAVITY_POTENTIAL_HPP_
