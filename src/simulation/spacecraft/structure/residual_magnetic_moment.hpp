/**
 * @file residual_magnetic_moment.hpp
 * @brief Definition of Residual Magnetic Moment (RMM)
 */

#ifndef S2E_SIMULATION_SPACECRAFT_STRUCTURE_RESIDUAL_MAGNETIC_MOMENT_HPP_
#define S2E_SIMULATION_SPACECRAFT_STRUCTURE_RESIDUAL_MAGNETIC_MOMENT_HPP_

#include <math_physics/math/vector.hpp>

namespace s2e::simulation {

/**
 * @class ResidualMagneticMoment
 * @brief Class for spacecraft RMM (Residual Magnetic Moment)
 */
class ResidualMagneticMoment {
 public:
  /**
   * @fn ResidualMagneticMoment
   * @brief Constructor
   */
  ResidualMagneticMoment(const Vector<3> constant_value_b_Am2_, const double random_walk_standard_deviation_Am2, const double random_walk_limit_Am2,
                         const double random_noise_standard_deviation_Am2);
  /**
   * @fn ~ResidualMagneticMoment
   * @brief Destructor
   */
  ~ResidualMagneticMoment(){};

  // Getter
  /**
   * @fn GetConstantValue_b_Am2
   * @brief Return Constant value of RMM at body frame [Am2]
   */
  inline const Vector<3>& GetConstantValue_b_Am2(void) const { return constant_value_b_Am2_; }
  /**
   * @fn GetRandomWalkStandardDeviation_Am2
   * @brief Return Random walk standard deviation of RMM [Am2]
   */
  inline const double& GetRandomWalkStandardDeviation_Am2(void) const { return random_walk_standard_deviation_Am2_; }
  /**
   * @fn GetRandomWalkLimit_Am2
   * @brief Random walk limit of RMM [Am2]
   */
  inline const double& GetRandomWalkLimit_Am2(void) const { return random_walk_limit_Am2_; }
  /**
   * @fn GetRandomNoiseStandardDeviation_Am2
   * @brief Standard deviation of white noise of RMM [Am2]
   */
  inline const double& GetRandomNoiseStandardDeviation_Am2(void) const { return random_noise_standard_deviation_Am2_; }

  // Setter
  /**
   * @fn SetRMMConst_b_Am2
   * @brief Set Constant value of RMM at body frame [Am2]
   * @param [in] rmm_const_b_Am2: Constant value of RMM at the body frame [Am2]
   */
  inline void SetRmmConstant_b_Am2(const math::Vector<3> rmm_const_b_Am2) { constant_value_b_Am2_ = rmm_const_b_Am2; }
  /**
   * @fn AddRMMConst_b_Am2
   * @brief Add Constant value of RMM at body frame [Am2]
   * @param [in] rmm_const_b_Am2: Constant value of RMM at the body frame [Am2]
   */
  inline void AddRmmConstant_b_Am2(const math::Vector<3> rmm_const_b_Am2) { constant_value_b_Am2_ += rmm_const_b_Am2; }

 private:
  math::Vector<3> constant_value_b_Am2_;              //!< Constant value of RMM at body frame [Am2]
  double random_walk_standard_deviation_Am2_;   //!< Random walk standard deviation of RMM [Am2]
  double random_walk_limit_Am2_;                //!< Random walk limit of RMM [Am2]
  double random_noise_standard_deviation_Am2_;  //!< Standard deviation of white noise of RMM [Am2]
};

} // namespace s2e::simulation

#endif  // S2E_SIMULATION_SPACECRAFT_STRUCTURE_RESIDUAL_MAGNETIC_MOMENT_HPP_