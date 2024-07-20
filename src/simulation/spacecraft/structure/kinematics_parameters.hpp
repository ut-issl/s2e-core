/**
 * @file kinematics_parameters.h
 * @brief Definition of Kinematics information
 */

#ifndef S2E_SIMULATION_SPACECRAFT_STRUCTURE_KINEMATICS_PARAMETERS_HPP_
#define S2E_SIMULATION_SPACECRAFT_STRUCTURE_KINEMATICS_PARAMETERS_HPP_

#include <math_physics/math/matrix.hpp>
#include <math_physics/math/vector.hpp>

/**
 * @class KinematicsParameters
 * @brief Class for spacecraft Kinematics information
 */
class KinematicsParameters {
 public:
  /**
   * @fn KinematicsParameters
   * @brief Constructor
   */
  KinematicsParameters(math::Vector<3> center_of_gravity_b_m, double mass_kg, math::Matrix<3, 3> inertia_tensor_b_kgm2);
  /**
   * @fn ~KinematicsParameters
   * @brief Destructor
   */
  ~KinematicsParameters() {};

  // Getter
  /**
   * @fn GetCenterOfGravity_b_m
   * @brief Return Position vector of center of gravity at body frame [m]
   */
  inline const math::Vector<3>& GetCenterOfGravity_b_m() const { return center_of_gravity_b_m_; }
  /**
   * @fn GetMass_kg
   * @brief Return Mass of the satellite [kg]
   */
  inline const double& GetMass_kg() const { return mass_kg_; }
  /**
   * @fn GetInertiaTensor_b_kgm2
   * @brief Return Inertia tensor at body frame [kgm2]
   */
  inline const math::Matrix<3, 3>& GetInertiaTensor_b_kgm2() const { return inertia_tensor_b_kgm2_; }

  // Setter
  /**
   * @fn SetCenterOfGravityVector_b_m
   * @brief Set center of gravity vector at the body frame [m]
   * @param [in] center_of_gravity_vector_b_m: Center of gravity vector at the body frame [m]
   */
  inline void SetCenterOfGravityVector_b_m(const math::Vector<3> center_of_gravity_vector_b_m) {
    center_of_gravity_b_m_ = center_of_gravity_vector_b_m;
  }
  /**
   * @fn SetMass_kg
   * @brief Set mass_kg of the satellite
   * @param [in] mass_kg: Mass of the satellite [kg]
   */
  inline void SetMass_kg(const double mass_kg) {
    if (mass_kg > 0.0) mass_kg_ = mass_kg;
  }
  /**
   * @fn AddMass_kg
   * @brief Add mass_kg of the satellite
   * @param [in] mass_kg: Mass of the satellite [kg]
   * @note Normally, this function is used to decrease the mass_kg due to the fuel consumption.
   */
  inline void AddMass_kg(const double mass_kg) {
    double temp_mass_kg = mass_kg_ + mass_kg;
    SetMass_kg(temp_mass_kg);
  }
  /**
   * @fn SetInertiaTensor_b_kgm2
   * @brief Inertia tensor at body frame
   * @param [in] inertia_tensor_b_kgm2: Inertia tensor at body frame [kgm2]
   */
  inline void SetInertiaTensor_b_kgm2(const math::Matrix<3, 3> inertia_tensor_b_kgm2) {
    // TODO add assertion check
    inertia_tensor_b_kgm2_ = inertia_tensor_b_kgm2;
  }

 private:
  math::Vector<3> center_of_gravity_b_m_;     //!< Position vector of center of gravity at body frame [m]
  double mass_kg_;                            //!< Mass of the satellite [kg]
  math::Matrix<3, 3> inertia_tensor_b_kgm2_;  //!< Inertia tensor at body frame [kgm2]
};

#endif  // S2E_SIMULATION_SPACECRAFT_STRUCTURE_KINEMATICS_PARAMETERS_HPP_
