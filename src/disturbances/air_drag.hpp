/**
 * @file air_drag.hpp
 * @brief Class to calculate the air drag disturbance force and torque
 */

#ifndef S2E_DISTURBANCES_AIR_DRAG_HPP_
#define S2E_DISTURBANCES_AIR_DRAG_HPP_

#include <vector>

#include "../environment/local/atmosphere.hpp"
#include "../interface/log_output/loggable.hpp"
#include "../library/math/vector.hpp"
#include "surface_force.hpp"

#pragma once
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
   * @param [in] wall_temperature_degC: Temperature of surfaces [degC]
   * @param [in] molecular_temperature_degC: Temperature of air molecular [degC]
   * @param [in] molecular_weight: Molecular weight
   * @param [in] is_calculation_enabled: Calculation flag
   */
  AirDrag(const vector<Surface>& surfaces, const libra::Vector<3>& center_of_gravity_b_m, const double wall_temperature_degC,
          const double molecular_temperature_degC, const double molecular_weight, const bool is_calculation_enabled = true);

  /**
   * @fn Update
   * @brief Override Updates function of SimpleDisturbance
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
  vector<double> Cn_;  //!< Coefficients for out-plane force
  vector<double> Ct_;  //!< Coefficients for in-plane force
  double rho_kg_m3_;   //!< Air density [kg/m^3]
  double Tw_;          //!< Temperature of surface [K]
  double Tm_;          //!< Temperature of atmosphere [K]
  double M_;           //!< Molecular weight [g/mol]

  /**
   * @fn CalcCoefficients
   * @brief Override CalcCoefficients function of SurfaceForce
   * @param [in] velocity_b_m_s: Spacecraft's velocity vector in the body frame [m/s]
   * @param [in] air_dens: Air density around the spacecraft [kg/m^3]
   */
  void CalcCoefficients(libra::Vector<3>& velocity_b_m_s, double air_dens);

  // internal function for calculation
  /**
   * @fn CalCnCt
   * @brief Calculate the Cn and Ct
   * @param [in] velocity_b_m_s: Spacecraft's velocity vector in the body frame [m/s]
   */
  void CalCnCt(libra::Vector<3>& velocity_b_m_s);
  /**
   * @fn funcPi
   * @brief Calculate The Pi function in the algorithm
   */
  double CalcFuncPi(double s);
  /**
   * @fn funcChi
   * @brief Calculate The Chi function in the algorithm
   */
  double CalcFuncChi(double s);
};

#endif  // S2E_DISTURBANCES_AIR_DRAG_HPP_
