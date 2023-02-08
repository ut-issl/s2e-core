/**
 * @file geopotential.hpp
 * @brief Class to calculate the high-order earth gravity acceleration
 */

#ifndef S2E_DISTURBANCES_GEOPOTENTIAL_H_
#define S2E_DISTURBANCES_GEOPOTENTIAL_H_

#include <string>

#include "../library/math/matrix_vector.hpp"
#include "../library/math/matrix.hpp"
#include "../library/math/Vector.hpp"
#include "../interface/log_output/loggable.hpp"
#include "acceleration_disturbance.hpp"

using libra::Matrix;
using libra::Vector;

/**
 * @class GeoPotential
 * @brief Class to calculate the high-order earth gravity acceleration
 */
class GeoPotential : public AccelerationDisturbance {
 public:
  /**
   * @fn GeoPotential
   * @brief Constructor
   */
  GeoPotential(const int degree, const std::string file_path);

  /**
   * @fn Update
   * @brief Override Updates function of SimpleDisturbance
   */
  virtual void Update(const LocalEnvironment &local_env, const Dynamics &dynamics);

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

  /**
   * @fn CalcAccelerationECEF
   * @brief Calculate the high-order earth gravity in the ECEF frame
   * @param [in] position_ecef: Position of the spacecraft in the ECEF fram [m]
   */
  void CalcAccelerationECEF(const Vector<3> &position_ecef);

  /**
   * @fn ReadCoefficientsEGM96
   * @brief Read the geo-potential coefficients for the EGM96 model
   * @param [in] file_name: Coefficient file name
   */
  bool ReadCoefficientsEGM96(std::string file_name);

 private:
  int degree_;                //!< Maximum degree setting to calculate the geo-potential
  vector<vector<double>> c_;  //!< Cosine coefficients
  vector<vector<double>> s_;  //!< Sine coefficients
  Vector<3> acc_ecef_;        //!< Calculated acceleration in the ECEF frame [m/s2]
  // calculation
  double r = 0.0;                    //!< Radius [m]
  double x = 0.0, y = 0.0, z = 0.0;  //!< Spacecraft position in ECEF frame [m]
  int n = 0, m = 0;                  //!< Degree and order

  /**
   * @fn v_w_nn_update
   * @brief Calculate V and W function for n = m
   */
  void v_w_nn_update(double *v_nn, double *w_nn, const double v_prev, const double w_prev);
  /**
   * @fn v_w_nm_update
   * @brief Calculate V and W function for n not equal m
   */
  void v_w_nm_update(double *v_nm, double *w_nm, const double v_prev, const double w_prev, const double v_prev2, const double w_prev2);

  // debug
  Vector<3> debug_pos_ecef_;  //!< Spacecraft position in ECEF frame [m]
  double time_ = 0.0;         //!< Calculation time [ms]
};

#endif  // S2E_DISTURBANCES_GEOPOTENTIAL_H_
