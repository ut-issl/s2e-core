/**
 * @file surface_force.hpp
 * @brief Base class for disturbances acting on a spacecraft surface (e.g., SRP, Air drag, etc)
 */

#pragma once

#ifndef S2E_DISTURBANCES_SURFACE_FORCE_HPP_
#define S2E_DISTURBANCES_SURFACE_FORCE_HPP_

#include <vector>

#include "../library/math/quaternion.hpp"
#include "../library/math/vector.hpp"
#include "../simulation/spacecraft/structure/surface.hpp"
#include "simple_disturbance.hpp"

/**
 * @class ThirdBodyGravity
 * @brief Class to calculate third body gravity disturbance
 */
class SurfaceForce : public SimpleDisturbance {
 public:
  /**
   * @fn SurfaceForce
   * @brief Constructor
   * @param [in] surfaces: Surface information of the spacecraft
   * @param [in] center_of_gravity_b_m: Center of gravity position at the body frame [m]
   * @param [in] is_calculation_enabled: Calculation flag
   */
  SurfaceForce(const vector<Surface>& surfaces, const libra::Vector<3>& center_of_gravity_b_m, const bool is_calculation_enabled = true);
  /**
   * @fn ~SurfaceForce
   * @brief Destructor
   */
  virtual ~SurfaceForce() {}

 protected:
  // Spacecraft Structure parameters
  const vector<Surface>& surfaces_;                //!< List of surfaces
  const libra::Vector<3>& center_of_gravity_b_m_;  //!< Position vector of the center of mass at body frame [m]

  // Internal calculated variables
  vector<double> normal_coefficients_;      //!< coefficients for out-plane force for each surface
  vector<double> tangential_coefficients_;  //!< coefficients for in-plane force for each surface
  vector<double> cos_theta_;  //!< cos(theta) for each surface (theta is the angle b/w normal vector and the direction of disturbance source)
  vector<double> sin_theta_;  //!< sin(theta) for each surface (theta is the angle b/w normal vector and the direction of disturbance source)

  // Functions
  /**
   * @fn CalcTorqueForce
   * @brief Calculate the torque and force
   * @param [in] input_direction_b: Direction of disturbance source at the body frame
   * @param [in] item: Parameter which decide the magnitude of the disturbances (e.g., Solar flux, air density)
   * @return Calculated disturbance torque in body frame [Nm]
   */
  libra::Vector<3> CalcTorqueForce(libra::Vector<3>& input_direction_b, double item);
  /**
   * @fn CalcTheta
   * @brief Calculate cosX and sinX
   * @param [in] input_direction_b: Direction of disturbance source at the body frame
   */
  void CalcTheta(libra::Vector<3>& input_direction_b);

  /**
   * @fn CalcCoefficients
   * @brief Pure virtual function to define the calculation of the disturbance coefficients
   * @param [in] input_direction_b: Direction of disturbance source at the body frame
   * @param [in] item: Parameter which decide the magnitude of the disturbances (e.g., Solar flux, air density)
   */
  virtual void CalcCoefficients(libra::Vector<3>& input_direction_b, double item) = 0;
};

#endif  // S2E_DISTURBANCES_SURFACE_FORCE_HPP_
