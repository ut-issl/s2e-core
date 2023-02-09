/**
 * @file magnetorquer.hpp
 * @brief Class to emulate magnetorquer
 */

#ifndef S2E_COMPONENTS_AOCS_MAGNETORQUER_HPP_
#define S2E_COMPONENTS_AOCS_MAGNETORQUER_HPP_

#include <environment/local/local_environment.hpp>
#include <interface/log_output/loggable.hpp>
#include <library/math/matrix.hpp>
#include <library/math/normal_randomization.hpp>
#include <library/math/quaternion.hpp>
#include <library/math/random_walk.hpp>
#include <library/math/vector.hpp>

#include "../base/component.hpp"

const size_t kMtqDim = 3;  //!< Dimension of magnetorquer

/**
 * @class MagTorquer
 * @brief Class to emulate magnetorquer
 */
class MagTorquer : public ComponentBase, public ILoggable {
 public:
  /**
   * @fn MagTorquer
   * @brief Constructor without power port
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_gen: Clock generator
   * @param [in] id : Actuator ID
   * @param [in] q_b2c: Quaternion from body frame to component frame
   * @param [in] scale_facter: Scale factor matrix
   * @param [in] max_c : Maximum magnetic moment in the component frame [Am2]
   * @param [in] min_c : Minimum magnetic moment in the component frame [Am2]
   * @param [in] bias_c : Constant bias noise in the component frame [Am2]
   * @param [in] rw_stepwidth_c : Step width for random walk dynamics [s]
   * @param [in] rw_stddev_c: Standard deviation of random walk noise in the component frame [Am2]
   * @param [in] rw_limit_c: Limit for random walk noise in the component frame [Am2]
   * @param [in] nr_stddev_c: Standard deviation for the normal random noise in the component frame [Am2]
   * @param [in] magnet: Geomagnetic environment
   */
  MagTorquer(const int prescaler, ClockGenerator* clock_gen, const int id, const libra::Quaternion& q_b2c,
             const libra::Matrix<kMtqDim, kMtqDim>& scale_facter, const libra::Vector<kMtqDim>& max_c, const libra::Vector<kMtqDim>& min_c,
             const libra::Vector<kMtqDim>& bias_c, double rw_stepwidth, const libra::Vector<kMtqDim>& rw_stddev_c,
             const libra::Vector<kMtqDim>& rw_limit_c, const libra::Vector<kMtqDim>& nr_stddev_c, const MagEnvironment* mag_env);
  /**
   * @fn MagTorquer
   * @brief Constructor with power port
   * @param [in] prescaler: Frequency scale factor for update
   * @param [in] clock_gen: Clock generator
   * @param [in] power_port: Power port
   * @param [in] id : Actuator ID
   * @param [in] q_b2c: Quaternion from body frame to component frame
   * @param [in] scale_facter: Scale factor matrix
   * @param [in] max_c : Maximum magnetic moment in the component frame [Am2]
   * @param [in] min_c : Minimum magnetic moment in the component frame [Am2]
   * @param [in] bias_c : Constant bias noise in the component frame [Am2]
   * @param [in] rw_stepwidth_c : Step width for random walk dynamics [s]
   * @param [in] rw_stddev_c: Standard deviation of random walk noise in the component frame [Am2]
   * @param [in] rw_limit_c: Limit for random walk noise in the component frame [Am2]
   * @param [in] nr_stddev_c: Standard deviation for the normal random noise in the component frame [Am2]
   * @param [in] magnet: Geomagnetic environment
   */
  MagTorquer(const int prescaler, ClockGenerator* clock_gen, PowerPort* power_port, const int id, const libra::Quaternion& q_b2c,
             const libra::Matrix<kMtqDim, kMtqDim>& scale_facter, const libra::Vector<kMtqDim>& max_c, const libra::Vector<kMtqDim>& min_c,
             const libra::Vector<kMtqDim>& bias_c, double rw_stepwidth, const libra::Vector<kMtqDim>& rw_stddev_c,
             const libra::Vector<kMtqDim>& rw_limit_c, const libra::Vector<kMtqDim>& nr_stddev_c, const MagEnvironment* mag_env);

  // Override functions for ComponentBase
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
   * @fn GetMagMoment_b
   * @brief Return output magnetic moment in the body fixed frame [Am2]
   */
  inline const libra::Vector<kMtqDim>& GetMagMoment_b(void) const { return mag_moment_b_; };
  /**
   * @fn GetTorque_b
   * @brief Return output torque in the body fixed frame [Nm]
   */
  inline const libra::Vector<kMtqDim>& GetTorque_b(void) const { return torque_b_; };

  /**
   * @fn SetMagMomentC
   * @brief Set output magnetic moment in the component frame [Am2]
   */
  inline void SetMagMomentC(const libra::Vector<kMtqDim> mag_moment_c) { mag_moment_c_ = mag_moment_c; };

 protected:
  const int id_ = 0;                              //!< Actuator ID
  const double knT2T = 1.0e-9;                    //!< Constant to convert nT to T
  libra::Vector<kMtqDim> torque_b_{0.0};          //!< Output torque in the body fixed frame [Nm]
  libra::Vector<kMtqDim> mag_moment_c_{0.0};      //!< Output output magnetic moment in the component frame [Am2]
  libra::Vector<kMtqDim> mag_moment_b_{0.0};      //!< Output output magnetic moment in the body fixed frame [Am2]
  libra::Quaternion q_b2c_{0.0, 0.0, 0.0, 1.0};   //!< Quaternion from body frame to component frame
  libra::Quaternion q_c2b_{0.0, 0.0, 0.0, 1.0};   //!< Quaternion from component frame to body frame
  libra::Matrix<kMtqDim, kMtqDim> scale_factor_;  //!< Scale factor matrix
  libra::Vector<kMtqDim> max_c_{100.0};           //!< Maximum magnetic moment in the component frame [Am2]
  libra::Vector<kMtqDim> min_c_{-100.0};          //!< Minimum magnetic moment in the component frame [Am2]
  libra::Vector<kMtqDim> bias_c_{0.0};            //!< Constant bias noise in the component frame [Am2]
  RandomWalk<kMtqDim> n_rw_c_;                    //!< Random walk noise
  libra::NormalRand nrs_c_[kMtqDim];              //!< Normal random noise

  const MagEnvironment* mag_env_;  //!< Geomagnetic environment

  /**
   * @fn CalcOutputTorque
   * @brief Calculate output torque
   * @return Output torque in the body fixed frame [Nm]
   */
  libra::Vector<kMtqDim> CalcOutputTorque(void);
};

#endif  // S2E_COMPONENTS_AOCS_MAGNETORQUER_HPP_
