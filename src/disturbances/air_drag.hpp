/**
 * @file air_drag.hpp
 * @brief Class to calculate the air drag disturbance force and torque
 */

#ifndef S2E_DISTURBANCES_AIR_DRAG_HPP_
#define S2E_DISTURBANCES_AIR_DRAG_HPP_

#include <vector>

#include "../environment/local/atmosphere.hpp"
#include "../library/logger/loggable.hpp"
#include "../library/math/quaternion.hpp"
#include "../library/math/vector.hpp"
#include "surface_force.hpp"

/**
 * @class AirDrag
 * @brief Class to calculate the air drag disturbance force and torque
 */
class AirDrag : public SurfaceForce {
 public:
  /**
   * @fn AirDrag
   * @brief Constructor
   * @param [in] surfaces: Surface information of the spacecraft
   * @param [in] center_of_gravity_b_m: Center of gravity position at the body frame [m]
   * @param [in] wall_temperature_K: Temperature of surfaces [K]
   * @param [in] molecular_temperature_K: Temperature of air molecular [K]
   * @param [in] molecular_weight_g_mol: Molecular weight [g/mol]
   * @param [in] is_calculation_enabled: Calculation flag
   */
  AirDrag(const std::vector<Surface>& surfaces, const libra::Vector<3>& center_of_gravity_b_m, const double wall_temperature_K,
          const double molecular_temperature_K, const double molecular_weight_g_mol, const bool is_calculation_enabled = true);

  /**
   * @fn Update
   * @brief Override Updates function of SimpleDisturbance
   * @param [in] local_environment: Local environment information
   * @param [in] dynamics: Dynamics information
   */
  virtual void Update(const LocalEnvironment& local_environment, const Dynamics& dynamics);

  // Override ILoggable
  /**
   * @fn GetLogHeader
   * @brief Override GetLogHeader function of ILoggable
   */
  virtual std::string GetLogHeader() const;
  /**
   * @fn GetLogValue
   * @brief Override GetLogValue function of ILoggable
   */
  virtual std::string GetLogValue() const;

 private:
  std::vector<double> cn_;          //!< Coefficients for out-plane force
  std::vector<double> ct_;          //!< Coefficients for in-plane force
  double wall_temperature_K_;       //!< Temperature of surface [K]
  double molecular_temperature_K_;  //!< Temperature of atmosphere [K]
  double molecular_weight_g_mol_;   //!< Molecular weight [g/mol]

  /**
   * @fn CalcCoefficients
   * @brief Override CalcCoefficients function of SurfaceForce
   * @param [in] velocity_b_m_s: Spacecraft's velocity vector in the body frame [m/s]
   * @param [in] air_density_kg_m3: Air density around the spacecraft [kg/m^3]
   */
  void CalcCoefficients(const libra::Vector<3>& velocity_b_m_s, const double air_density_kg_m3);

  // internal function for calculation
  /**
   * @fn CalcCnCt
   * @brief Calculate the Cn and Ct
   * @param [in] velocity_b_m_s: Spacecraft's velocity vector in the body frame [m/s]
   */
  void CalcCnCt(const libra::Vector<3>& velocity_b_m_s);
  /**
   * @fn CalcFunctionPi
   * @brief Calculate The Pi function in the algorithm
   * @param [in] s: Independent variable of the Pi function
   */
  double CalcFunctionPi(const double s);
  /**
   * @fn CalcFunctionChi
   * @brief Calculate The Chi function in the algorithm
   * @param [in] s: Independent variable of the Chi function
   */
  double CalcFunctionChi(const double s);
};

/**
 * @fn InitAirDrag
 * @brief Initialize AirDrag class
 * @param [in] initialize_file_path: Initialize file path
 * @param [in] surfaces: surface information of the spacecraft
 * @param [in] center_of_gravity_b_m: Center of gravity position vector at body frame [m]
 */
AirDrag InitAirDrag(const std::string initialize_file_path, const std::vector<Surface>& surfaces, const Vector<3>& center_of_gravity_b_m);

#endif  // S2E_DISTURBANCES_AIR_DRAG_HPP_
