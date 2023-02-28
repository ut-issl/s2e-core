/**
 * @file residual_magnetic_moment.hpp
 * @brief Definition of Residual Magnetic Moment (RMM)
 */

#ifndef S2E_SIMULATION_SPACECRAFT_STRUCTURE_RESIDUAL_MAGNETIC_MOMENT_HPP_
#define S2E_SIMULATION_SPACECRAFT_STRUCTURE_RESIDUAL_MAGNETIC_MOMENT_HPP_

#include <library/math/vector.hpp>
using libra::Vector;

/**
 * @class RMMParams
 * @brief Class for spacecraft RMM (Residual Magnetic Moment)
 */
class RMMParams {
 public:
  /**
   * @fn RMMParams
   * @brief Constructor
   */
  RMMParams(Vector<3> rmm_const_b, double rmm_rwdev, double rmm_rwlimit, double rmm_wnvar);
  /**
   * @fn ~RMMParams
   * @brief Destructor
   */
  ~RMMParams(){};

  // Getter
  /**
   * @fn GetRMMConst_b
   * @brief Return Constant value of RMM at body frame [Am2]
   */
  inline const Vector<3>& GetRMMConst_b(void) const { return constant_value_b_Am2_; }
  /**
   * @fn GetRMMRWDev
   * @brief Return Random walk standard deviation of RMM [Am2]
   */
  inline const double& GetRMMRWDev(void) const { return random_walk_standard_deviation_Am2; }
  /**
   * @fn GetRMMRWDev
   * @brief Random walk limit of RMM [Am2]
   */
  inline const double& GetRMMRWLimit(void) const { return random_walk_limit_Am2; }
  /**
   * @fn GetRMMRWDev
   * @brief Standard deviation of white noise of RMM [Am2]
   */
  inline const double& GetRMMWNVar(void) const { return random_noise_standard_deviation_Am2; }

  // Setter
  /**
   * @fn SetRMMConst_b_Am2
   * @brief Set Constant value of RMM at body frame [Am2]
   * @param [in] rmm_const_b_Am2: Constant value of RMM at the body frame [Am2]
   */
  inline void SetRmmConstant_b_Am2(const Vector<3> rmm_const_b_Am2) { constant_value_b_Am2_ = rmm_const_b_Am2; }
  /**
   * @fn AddRMMConst_b_Am2
   * @brief Add Constant value of RMM at body frame [Am2]
   * @param [in] rmm_const_b_Am2: Constant value of RMM at the body frame [Am2]
   */
  inline void AddRmmConstant_b_Am2(const Vector<3> rmm_const_b_Am2) { constant_value_b_Am2_ += rmm_const_b_Am2; }

 private:
  Vector<3> constant_value_b_Am2_;             //!< Constant value of RMM at body frame [Am2]
  double random_walk_standard_deviation_Am2;   //!< Random walk standard deviation of RMM [Am2]
  double random_walk_limit_Am2;                //!< Random walk limit of RMM [Am2]
  double random_noise_standard_deviation_Am2;  //!< Standard deviation of white noise of RMM [Am2]
};

#endif  // S2E_SIMULATION_SPACECRAFT_STRUCTURE_RESIDUAL_MAGNETIC_MOMENT_HPP_