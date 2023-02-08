/**
 * @file KeplerOrbit.h
 * @brief Class to calculate Kepler orbit calculation
 */
#pragma once
#include "../math/Matrix.hpp"
#include "../math/Vector.hpp"
#include "./OrbitalElements.h"

/**
 * @class KeplerOrbit
 * @brief Class to calculate Kepler orbit calculation
 */
class KeplerOrbit {
 public:
  /**
   * @fn KeplerOrbit
   * @brief Default Constructor
   */
  KeplerOrbit();
  /**
   * @fn KeplerOrbit
   * @brief Constructor
   * @param [in] mu_m3_s2: Gravity constant of the center body [m3/s2]
   * @param [in] oe: Orbital elements
   */
  KeplerOrbit(const double mu_m3_s2, const OrbitalElements oe);
  /**
   * @fn ~KeplerOrbit
   * @brief Destructor
   */
  ~KeplerOrbit();

  /**
   * @fn CalcPosVel
   * @brief Calculate position and velocity with Kepler orbit propagation
   * @param [in] time_jday: Time expressed as Julian day [day]
   */
  void CalcPosVel(double time_jday);

  /**
   * @fn GetPosition_i_m
   * @brief Return position vector in the inertial frame [m]
   */
  inline const libra::Vector<3> GetPosition_i_m() const { return position_i_m_; }
  /**
   * @fn GetVelocity_i_m_s
   * @brief Return velocity vector in the inertial frame [m/s]
   */
  inline const libra::Vector<3> GetVelocity_i_m_s() const { return velocity_i_m_s_; }

 protected:
  libra::Vector<3> position_i_m_;    //!< Position vector in the inertial frame [m]
  libra::Vector<3> velocity_i_m_s_;  //!< Velocity vector in the inertial frame [m/s]

 private:
  double mu_m3_s2_;                       //!< Gravity constant of the center body [m3/s2]
  OrbitalElements oe_;                    //!< Orbital elements
  double mean_motion_rad_s_;              //!< Mean motion of the orbit [rad/s]
  libra::Matrix<3, 3> dcm_inplane_to_i_;  //!< Direction cosine matrix from the in-plane frame to the inertial frame

  /**
   * @fn CalcConstKeplerMotion
   * @brief Calculate constants for kepler motion calculation
   */
  void CalcConstKeplerMotion();
  /**
   * @fn SolveKeplerFirstOrder
   * @brief Solve Kepler Equation with the first order approximation
   * @param [in] eccentricity: Eccentricity
   * @param [in] mean_anomaly_rad: Mean motion of the orbit [rad/s]
   * @param [in] angle_limit_rad: Limit of angle error for the approximation
   * @param [in] iteration_limit: Limit of iteration
   */
  double SolveKeplerFirstOrder(const double eccentricity, const double mean_anomaly_rad, const double angle_limit_rad, const int iteration_limit);
};
