/**
 * @file geopotential.hpp
 * @brief Class to calculate the high-order earth gravity acceleration
 */

#ifndef S2E_DISTURBANCES_GEOPOTENTIAL_HPP_
#define S2E_DISTURBANCES_GEOPOTENTIAL_HPP_

#include <string>

#include "../library/logger/loggable.hpp"
#include "../library/math/matrix.hpp"
#include "../library/math/matrix_vector.hpp"
#include "../library/math/vector.hpp"
#include "acceleration_disturbance.hpp"

/**
 * @class GeoPotential
 * @brief Class to calculate the high-order earth gravity acceleration
 */
class GeoPotential : public AccelerationDisturbance {
 public:
  /**
   * @fn GeoPotential
   * @brief Constructor
   * @param [in] degree: Maximum degree setting to calculate the geo-potential
   * @param [in] file_path: EGM96 coefficients file path
   * @param [in] is_calculation_enabled: Calculation flag
   */
  GeoPotential(const int degree, const std::string file_path, const bool is_calculation_enabled = true);

  /**
   * @fn Update
   * @brief Override Updates function of SimpleDisturbance
   * @param [in] local_environment: Local environment information
   * @param [in] dynamics: Dynamics information
   */
  virtual void Update(const LocalEnvironment &local_environment, const Dynamics &dynamics);

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
  int degree_;                        //!< Maximum degree setting to calculate the geo-potential
  int n_ = 0, m_ = 0;                 //!< Degree and order (FIXME: follow naming rule)
  vector<vector<double>> c_;          //!< Cosine coefficients
  vector<vector<double>> s_;          //!< Sine coefficients
  Vector<3> acceleration_ecef_m_s2_;  //!< Calculated acceleration in the ECEF frame [m/s2]

  // calculation
  double radius_m_ = 0.0;                                    //!< Radius [m]
  double ecef_x_m_ = 0.0, ecef_y_m_ = 0.0, ecef_z_m_ = 0.0;  //!< Spacecraft position in ECEF frame [m]

  // debug
  libra::Vector<3> debug_pos_ecef_m_;  //!< Spacecraft position in ECEF frame [m]
  double time_ms_ = 0.0;               //!< Calculation time [ms]

  /**
   * @fn CalcAccelerationEcef
   * @brief Calculate the high-order earth gravity in the ECEF frame
   * @param [in] position_ecef_m: Position of the spacecraft in the ECEF fram [m]
   */
  void CalcAccelerationEcef(const Vector<3> &position_ecef_m);

  /**
   * @fn ReadCoefficientsEgm96
   * @brief Read the geo-potential coefficients for the EGM96 model
   * @param [in] file_name: Coefficient file name
   */
  bool ReadCoefficientsEgm96(std::string file_name);

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

#endif  // S2E_DISTURBANCES_GEOPOTENTIAL_HPP_
