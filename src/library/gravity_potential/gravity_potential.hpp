/**
 * @file gravity_potential.hpp
 * @brief Class to calculate gravity potential
 */

#ifndef S2E_LIBRARY_GRAVITY_POTENTIAL_GRAVITY_POTENTIAL_HPP_
#define S2E_LIBRARY_GRAVITY_POTENTIAL_GRAVITY_POTENTIAL_HPP_

#include <environment/global/physical_constants.hpp>
#include <vector>

#include "../math/vector.hpp"

/**
 * @class GravityPotential
 * @brief Class to calculate gravity potential
 */
class GravityPotential {
 public:
  /**
   * @fn GravityPotential
   * @brief Constructor
   * @param [in] degree: Maximum degree setting to calculate the geo-potential
   */
  GravityPotential(const size_t degree, const std::vector<std::vector<double>> cosine_coefficients,
                   const std::vector<std::vector<double>> sine_coefficients,
                   const double gravity_constants_m3_s2 = environment::earth_gravitational_constant_m3_s2,
                   const double center_body_radius_m = environment::earth_equatorial_radius_m);

  ~GravityPotential() {}

  /**
   * @fn CalcAccelerationEcef
   * @brief Calculate the high-order earth gravity in the ECEF frame
   * @param [in] position_ecef_m: Position of the spacecraft in the ECEF fram [m]
   */
  libra::Vector<3> CalcAccelerationEcef(const libra::Vector<3> &position_ecef_m);

 private:
  size_t degree_;                       //!< Maximum degree setting to calculate the geo-potential
  size_t n_ = 0, m_ = 0;                //!< Degree and order (FIXME: follow naming rule)
  std::vector<std::vector<double>> c_;  //!< Cosine coefficients
  std::vector<std::vector<double>> s_;  //!< Sine coefficients

  const double gravity_constants_m3_s2_;
  const double center_body_radius_m_;
  // calculation
  double radius_m_ = 0.0;                                    //!< Radius [m]
  double ecef_x_m_ = 0.0, ecef_y_m_ = 0.0, ecef_z_m_ = 0.0;  //!< Spacecraft position in ECEF frame [m]

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

#endif  // S2E_LIBRARY_GRAVITY_POTENTIAL_GRAVITY_POTENTIAL_HPP_
