/**
 * @file kinematics_parameters.h
 * @brief Definition of Kinematics information
 */

#ifndef S2E_SIMULATION_SPACECRAFT_STRUCTURE_KINEMATICS_PARAMETERS_HPP_
#define S2E_SIMULATION_SPACECRAFT_STRUCTURE_KINEMATICS_PARAMETERS_HPP_

#include <library/math/matrix.hpp>
#include <library/math/vector.hpp>

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
  KinematicsParams(libra::Vector<3> cg_b, double mass, libra::Matrix<3, 3> inertia_tensor);
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
  inline const libra::Vector<3>& GetCGb() const { return cg_b_; }
  /**
   * @fn GetMass
   * @brief Return Mass of the satellite [kg]
   */
  inline const double& GetMass() const { return mass_; }
  /**
   * @fn GetInertiaTensor
   * @brief Return Inertia tensor at body frame [kgm2]
   */
  inline const libra::Matrix<3, 3>& GetInertiaTensor() const { return inertia_tensor_; }

  // Setter
  /**
   * @fn SetCenterOfGravityVector_b_m
   * @brief Set center of gravity vector at the body frame [m]
   * @param [in] center_of_gravity_vector_b_m: Center of gravity vector at the body frame [m]
   */
  inline void SetCenterOfGravityVector_b_m(const libra::Vector<3> center_of_gravity_vector_b_m) { cg_b_ = center_of_gravity_vector_b_m; }
  /**
   * @fn SetMass_kg
   * @brief Set mass of the satellite
   * @param [in] mass_kg: Mass of the satellite [kg]
   */
  inline void SetMass_kg(const double mass_kg) {
    if (mass_kg > 0.0) mass_ = mass_kg;
  }
  /**
   * @fn AddMass_kg
   * @brief Add mass of the satellite
   * @param [in] mass_kg: Mass of the satellite [kg]
   * @note Normally, this function is used to decrease the mass due to the fuel consumption.
   */
  inline void AddMass_kg(const double mass_kg) {
    double temp_mass_kg = mass_ + mass_kg;
    SetMass_kg(temp_mass_kg);
  }
  /**
   * @fn SetInertiaTensor_b_kgm2
   * @brief Inertia tensor at body frame
   * @param [in] inertia_tensor_b_kgm2: Inertia tensor at body frame [kgm2]
   */
  inline void SetInertiaTensor_b_kgm2(const libra::Matrix<3, 3> inertia_tensor_b_kgm2) {
    // TODO add assertion check
    inertia_tensor_ = inertia_tensor_b_kgm2;
  }

 private:
  libra::Vector<3> cg_b_;               //!< Position vector of center of gravity at body frame [m]
  double mass_;                         //!< Mass of the satellite [kg]
  libra::Matrix<3, 3> inertia_tensor_;  //!< Inertia tensor at body frame [kgm2]
};

#endif  // S2E_SIMULATION_SPACECRAFT_STRUCTURE_KINEMATICS_PARAMETERS_HPP_
