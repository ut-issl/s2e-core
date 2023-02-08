/**
 * @file surface_force.hpp
 * @brief Base class for disturbances acting on a spacecraft surface (e.g., SRP, Air drag, etc)
 */

#pragma once

#ifndef S2E_DISTURBANCES_SURFACE_FORCE_H_
#define S2E_DISTURBANCES_SURFACE_FORCE_H_

#include "../library/math/quaternion.hpp"
#include "../library/math/vector.hpp"
#include "../simulation/spacecraft/structure/surface.hpp"
#include "simple_disturbance.hpp"
using libra::Quaternion;
using libra::Vector;

#include <vector>

/**
 * @class ThirdBodyGravity
 * @brief Class to calculate third body gravity disturbance
 */
class SurfaceForce : public SimpleDisturbance {
 public:
  /**
   * @fn SurfaceForce
   * @brief Constructor
   */
  SurfaceForce(const vector<Surface>& surfaces, const Vector<3>& cg_b);
  /**
   * @fn ~SurfaceForce
   * @brief Destructor
   */
  virtual ~SurfaceForce() {}

 protected:
  // Spacecraft Structure parameters
  const vector<Surface>& surfaces_;  //!< List of surfaces
  const Vector<3>& cg_b_;            //!< Position vector of the center of mass at body frame [m]

  // Internal calculated variables
  vector<double> normal_coef_;      //!< coefficients for out-plane force for each surface
  vector<double> tangential_coef_;  //!< coefficients for in-plane force for each surface
  vector<double> cosX;              //!< cos(X) for each surface (X is the angle b/w normal vector and the direction of disturbance source)
  vector<double> sinX;              //!< sin(X) for each surface (X is the angle b/w normal vector and the direction of disturbance source)

  // Functions
  /**
   * @fn CalcTorqueForce
   * @brief Calculate the torque and force
   * @param [in] input_b: Direction of disturbance source at the body frame
   * @param [in] item: Parameter which decide the magnitude of the disturbances (e.g., Solar flux, air density)
   * @return Calculated disturbance torque in body frame [Nm]
   */
  Vector<3> CalcTorqueForce(Vector<3>& input_b, double item);
  /**
   * @fn CalcTheta
   * @brief Calculate cosX and sinX
   * @param [in] input_b: Direction of disturbance source at the body frame
   */
  void CalcTheta(Vector<3>& input_b);

  /**
   * @fn CalcCoef
   * @brief Pure virtual function to define the calculation of the disturbance coefficients
   * @param [in] input_b: Direction of disturbance source at the body frame
   * @param [in] item: Parameter which decide the magnitude of the disturbances (e.g., Solar flux, air density)
   */
  virtual void CalcCoef(Vector<3>& input_b, double item) = 0;
};

#endif  // S2E_DISTURBANCES_SURFACE_FORCE_H_
