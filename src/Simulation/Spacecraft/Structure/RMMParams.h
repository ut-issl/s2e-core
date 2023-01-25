/**
 * @file RMMParams.h
 * @brief Definition of RMM (Residual Magnetic Moment)
 */

#pragma once
#include <Library/math/Vector.hpp>
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
  inline const Vector<3>& GetRMMConst_b(void) const { return rmm_const_b_; }
  /**
   * @fn GetRMMRWDev
   * @brief Return Random walk standard deviation of RMM [Am2]
   */
  inline const double& GetRMMRWDev(void) const { return rmm_rwdev_; }
  /**
   * @fn GetRMMRWDev
   * @brief Random walk limit of RMM [Am2]
   */
  inline const double& GetRMMRWLimit(void) const { return rmm_rwlimit_; }
  /**
   * @fn GetRMMRWDev
   * @brief Standard deviation of white noise of RMM [Am2]
   */
  inline const double& GetRMMWNVar(void) const { return rmm_wnvar_; }

  // Setter
  /**
   * @fn SetRMMConst_b_Am2
   * @brief Set Constant value of RMM at body frame [Am2]
   * @param [in] rmm_const_b_Am2: Constant value of RMM at the body frame [Am2]
   */
  inline void SetRMMConst_b_Am2(const Vector<3> rmm_const_b_Am2) { rmm_const_b_ = rmm_const_b_Am2; }

 private:
  Vector<3> rmm_const_b_;  //!< Constant value of RMM at body frame [Am2]
  double rmm_rwdev_;       //!< Random walk standard deviation of RMM [Am2]
  double rmm_rwlimit_;     //!< Random walk limit of RMM [Am2]
  double rmm_wnvar_;       //!< Standard deviation of white noise of RMM [Am2]
};
