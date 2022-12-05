/**
 * @file KinematicsParams.h
 * @brief Definition of Kinematics information
 */

#pragma once

#include <Library/math/Matrix.hpp>
#include <Library/math/Vector.hpp>
using libra::Matrix;
using libra::Vector;

/**
 * @class KinematicsParams
 * @brief Class for spacecraft Kinematics information
 */
class KinematicsParams {
 public:
  /**
   * @fn KinematicsParams
   * @brief Constructor
   */
  KinematicsParams(Vector<3> cg_b, double mass, Matrix<3, 3> inertia_tensor);
  /**
   * @fn ~KinematicsParams
   * @brief Destructor
   */
  ~KinematicsParams(){};

  // Getter
  /**
   * @fn GetCGb
   * @brief Return Position vector of center of gravity at body frame [m]
   */
  inline const Vector<3>& GetCGb() const { return cg_b_; }
  /**
   * @fn GetMass
   * @brief Return Mass of the satellite [kg]
   */
  inline const double& GetMass() const { return mass_; }
  /**
   * @fn GetInertiaTensor
   * @brief Return Inertia tensor at body frame [kgm2]
   */
  inline const Matrix<3, 3>& GetInertiaTensor() const { return inertia_tensor_; }

 private:
  Vector<3> cg_b_;               //!< Position vector of center of gravity at body frame [m]
  double mass_;                  //!< Mass of the satellite [kg]
  Matrix<3, 3> inertia_tensor_;  //!< Inertia tensor at body frame [kgm2]
};
