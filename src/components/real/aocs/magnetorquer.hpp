/**
 * @file magnetorquer.hpp
 * @brief Class to emulate magnetorquer
 */

#ifndef S2E_COMPONENTS_REAL_AOCS_MAGNETORQUER_HPP_
#define S2E_COMPONENTS_REAL_AOCS_MAGNETORQUER_HPP_

#include <environment/local/local_environment.hpp>
#include <library/logger/loggable.hpp>
#include <library/math/matrix.hpp>
#include <library/math/quaternion.hpp>
#include <library/math/vector.hpp>
#include <library/randomization/normal_randomization.hpp>
#include <library/randomization/random_walk.hpp>

#include "../../base/component.hpp"

const size_t kMtqDim = 3;  //!< Dimension of magnetorquer

/**
 * @class MagTorquer
 * @brief Class to emulate magnetorquer
 */
class MagTorquer : public Component, public ILoggable {
 public:
  /**
   * @fn MagTorquer
   * @brief Constructor without power port
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_generator: Clock generator
   * @param [in] component_id : Actuator ID
   * @param [in] quaternion_b2c: Quaternion from body frame to component frame
   * @param [in] scale_factor: Scale factor matrix
   * @param [in] max_magnetic_moment_c_Am2 : Maximum magnetic moment in the component frame [Am2]
   * @param [in] min_magnetic_moment_c_Am2 : Minimum magnetic moment in the component frame [Am2]
   * @param [in] bias_noise_c_Am2_ : Constant bias noise in the component frame [Am2]
   * @param [in] random_walk_step_width_s : Step width for random walk dynamics [s]
   * @param [in] random_walk_standard_deviation_c_Am2: Standard deviation of random walk noise in the component frame [Am2]
   * @param [in] random_walk_limit_c_Am2: Limit for random walk noise in the component frame [Am2]
   * @param [in] normal_random_standard_deviation_c_Am2: Standard deviation for the normal random noise in the component frame [Am2]
   * @param [in] geomagnetic_field: Geomagnetic environment
   */
  MagTorquer(const int prescaler, ClockGenerator* clock_generator, const int component_id, const libra::Quaternion& quaternion_b2c,
             const libra::Matrix<kMtqDim, kMtqDim>& scale_factor, const libra::Vector<kMtqDim>& max_magnetic_moment_c_Am2,
             const libra::Vector<kMtqDim>& min_magnetic_moment_c_Am2, const libra::Vector<kMtqDim>& bias_noise_c_Am2_,
             double random_walk_step_width_s, const libra::Vector<kMtqDim>& random_walk_standard_deviation_c_Am2,
             const libra::Vector<kMtqDim>& random_walk_limit_c_Am2, const libra::Vector<kMtqDim>& normal_random_standard_deviation_c_Am2,
             const GeomagneticField* geomagnetic_field);
  /**
   * @fn MagTorquer
   * @brief Constructor with power port
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_generator: Clock generator
   * @param [in] power_port: Power port
   * @param [in] component_id : Actuator ID
   * @param [in] quaternion_b2c: Quaternion from body frame to component frame
   * @param [in] scale_factor: Scale factor matrix
   * @param [in] max_magnetic_moment_c_Am2 : Maximum magnetic moment in the component frame [Am2]
   * @param [in] min_magnetic_moment_c_Am2 : Minimum magnetic moment in the component frame [Am2]
   * @param [in] bias_noise_c_Am2_ : Constant bias noise in the component frame [Am2]
   * @param [in] random_walk_step_width_s : Step width for random walk dynamics [s]
   * @param [in] random_walk_standard_deviation_c_Am2: Standard deviation of random walk noise in the component frame [Am2]
   * @param [in] random_walk_limit_c_Am2: Limit for random walk noise in the component frame [Am2]
   * @param [in] normal_random_standard_deviation_c_Am2: Standard deviation for the normal random noise in the component frame [Am2]
   * @param [in] geomagnetic_field: Geomagnetic environment
   */
  MagTorquer(const int prescaler, ClockGenerator* clock_generator, PowerPort* power_port, const int component_id,
             const libra::Quaternion& quaternion_b2c, const libra::Matrix<kMtqDim, kMtqDim>& scale_factor,
             const libra::Vector<kMtqDim>& max_magnetic_moment_c_Am2, const libra::Vector<kMtqDim>& min_magnetic_moment_c_Am2,
             const libra::Vector<kMtqDim>& bias_noise_c_Am2_, double random_walk_step_width_s,
             const libra::Vector<kMtqDim>& random_walk_standard_deviation_c_Am2, const libra::Vector<kMtqDim>& random_walk_limit_c_Am2,
             const libra::Vector<kMtqDim>& normal_random_standard_deviation_c_Am2, const GeomagneticField* geomagnetic_field);

  // Override functions for Component
  /**
   * @fn MainRoutine
   * @brief Main routine to output torque
   */
  void MainRoutine(int count) override;
  /**
   * @fn PowerOffRoutine
   * @brief Power off routine to stop actuation
   */
  void PowerOffRoutine() override;

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
   * @fn GetOutputTorque_b_Nm
   * @brief Return output torque in the body fixed frame [Nm]
   */
  inline const libra::Vector<kMtqDim>& GetOutputTorque_b_Nm(void) const { return torque_b_Nm_; };

  /**
   * @fn SetOutputMagneticMoment_c_Am2
   * @brief Set output magnetic moment in the component frame [Am2]
   */
  inline void SetOutputMagneticMoment_c_Am2(const libra::Vector<kMtqDim> mag_moment_c) { output_magnetic_moment_c_Am2_ = mag_moment_c; };

  /**
   * @fn SetOutputMagneticMoment_b_Am2
   * @brief Return output magnetic moment in the body fixed frame [Am2]
   */
  inline const libra::Vector<kMtqDim>& SetOutputMagneticMoment_b_Am2(void) const { return output_magnetic_moment_b_Am2_; };

 protected:
  const int component_id_ = 0;                                //!< Actuator ID
  const double kConvertNanoT2T = 1.0e-9;                      //!< Constant to convert nT to T
  libra::Vector<kMtqDim> torque_b_Nm_{0.0};                   //!< Output torque in the body fixed frame [Nm]
  libra::Vector<kMtqDim> output_magnetic_moment_c_Am2_{0.0};  //!< Output output magnetic moment in the component frame [Am2]
  libra::Vector<kMtqDim> output_magnetic_moment_b_Am2_{0.0};  //!< Output output magnetic moment in the body fixed frame [Am2]
  libra::Quaternion quaternion_b2c_{0.0, 0.0, 0.0, 1.0};      //!< Quaternion from body frame to component frame
  libra::Quaternion quaternion_c2b_{0.0, 0.0, 0.0, 1.0};      //!< Quaternion from component frame to body frame
  libra::Matrix<kMtqDim, kMtqDim> scale_factor_;              //!< Scale factor matrix
  libra::Vector<kMtqDim> max_magnetic_moment_c_Am2_{100.0};   //!< Maximum magnetic moment in the component frame [Am2]
  libra::Vector<kMtqDim> min_magnetic_moment_c_Am2_{-100.0};  //!< Minimum magnetic moment in the component frame [Am2]

  libra::Vector<kMtqDim> bias_noise_c_Am2_{0.0};   //!< Constant bias noise in the component frame [Am2]
  RandomWalk<kMtqDim> random_walk_c_Am2_;          //!< Random walk noise
  libra::NormalRand random_noise_c_Am2_[kMtqDim];  //!< Normal random noise

  const GeomagneticField* geomagnetic_field_;  //!< Geomagnetic environment

  /**
   * @fn CalcOutputTorque
   * @brief Calculate output torque
   * @return Output torque in the body fixed frame [Nm]
   */
  libra::Vector<kMtqDim> CalcOutputTorque(void);
};

#endif  // S2E_COMPONENTS_REAL_AOCS_MAGNETORQUER_HPP_
