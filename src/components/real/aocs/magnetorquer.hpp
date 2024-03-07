/**
 * @file magnetorquer.hpp
 * @brief Class to emulate magnetorquer
 */

#ifndef S2E_COMPONENTS_REAL_AOCS_MAGNETORQUER_HPP_
#define S2E_COMPONENTS_REAL_AOCS_MAGNETORQUER_HPP_

#include <environment/local/local_environment.hpp>
#include <logger/loggable.hpp>
#include <math_physics/math/matrix.hpp>
#include <math_physics/math/quaternion.hpp>
#include <math_physics/math/vector.hpp>
#include <math_physics/randomization/normal_randomization.hpp>
#include <math_physics/randomization/random_walk.hpp>

#include "../../base/component.hpp"

const size_t kMtqDimension = 3;  //!< Dimension of magnetorquer

/**
 * @class Magnetorquer
 * @brief Class to emulate magnetorquer
 */
class Magnetorquer : public Component, public ILoggable {
 public:
  /**
   * @fn Magnetorquer
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
  Magnetorquer(const int prescaler, ClockGenerator* clock_generator, const int component_id, const libra::Quaternion& quaternion_b2c,
               const libra::Matrix<kMtqDimension, kMtqDimension>& scale_factor, const libra::Vector<kMtqDimension>& max_magnetic_moment_c_Am2,
               const libra::Vector<kMtqDimension>& min_magnetic_moment_c_Am2, const libra::Vector<kMtqDimension>& bias_noise_c_Am2_,
               double random_walk_step_width_s, const libra::Vector<kMtqDimension>& random_walk_standard_deviation_c_Am2,
               const libra::Vector<kMtqDimension>& random_walk_limit_c_Am2,
               const libra::Vector<kMtqDimension>& normal_random_standard_deviation_c_Am2, const GeomagneticField* geomagnetic_field);
  /**
   * @fn Magnetorquer
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
  Magnetorquer(const int prescaler, ClockGenerator* clock_generator, PowerPort* power_port, const int component_id,
               const libra::Quaternion& quaternion_b2c, const libra::Matrix<kMtqDimension, kMtqDimension>& scale_factor,
               const libra::Vector<kMtqDimension>& max_magnetic_moment_c_Am2, const libra::Vector<kMtqDimension>& min_magnetic_moment_c_Am2,
               const libra::Vector<kMtqDimension>& bias_noise_c_Am2_, double random_walk_step_width_s,
               const libra::Vector<kMtqDimension>& random_walk_standard_deviation_c_Am2, const libra::Vector<kMtqDimension>& random_walk_limit_c_Am2,
               const libra::Vector<kMtqDimension>& normal_random_standard_deviation_c_Am2, const GeomagneticField* geomagnetic_field);

  // Override functions for Component
  /**
   * @fn MainRoutine
   * @brief Main routine to output torque
   */
  void MainRoutine(const int time_count) override;
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
  virtual std::string GetLogHeader() const override;
  /**
   * @fn GetLogValue
   * @brief Override GetLogValue function of ILoggable
   */
  virtual std::string GetLogValue() const override;

  /**
   * @fn GetOutputTorque_b_Nm
   * @brief Return output torque in the body fixed frame [Nm]
   */
  inline const libra::Vector<kMtqDimension>& GetOutputTorque_b_Nm(void) const { return torque_b_Nm_; };

  /**
   * @fn SetOutputMagneticMoment_c_Am2
   * @brief Set output magnetic moment in the component frame [Am2]
   */
  inline void SetOutputMagneticMoment_c_Am2(const libra::Vector<kMtqDimension> mag_moment_c) { output_magnetic_moment_c_Am2_ = mag_moment_c; };

  /**
   * @fn SetOutputMagneticMoment_b_Am2
   * @brief Return output magnetic moment in the body fixed frame [Am2]
   */
  inline const libra::Vector<kMtqDimension>& SetOutputMagneticMoment_b_Am2(void) const { return output_magnetic_moment_b_Am2_; };

  /**
   * @fn GetOutputMagneticMoment_b_Am2
   * @brief Return output magnetic moment in the body fixed frame [Am2]
   */
  inline const libra::Vector<kMtqDimension>& GetOutputMagneticMoment_c_Am2(void) const { return output_magnetic_moment_c_Am2_; };

 protected:
  const int component_id_ = 0;                                      //!< Actuator ID
  const double kConvertNanoT2T = 1.0e-9;                            //!< Constant to convert nT to T
  libra::Vector<kMtqDimension> torque_b_Nm_{0.0};                   //!< Output torque in the body fixed frame [Nm]
  libra::Vector<kMtqDimension> output_magnetic_moment_c_Am2_{0.0};  //!< Output output magnetic moment in the component frame [Am2]
  libra::Vector<kMtqDimension> output_magnetic_moment_b_Am2_{0.0};  //!< Output output magnetic moment in the body fixed frame [Am2]
  libra::Quaternion quaternion_b2c_{0.0, 0.0, 0.0, 1.0};            //!< Quaternion from body frame to component frame
  libra::Quaternion quaternion_c2b_{0.0, 0.0, 0.0, 1.0};            //!< Quaternion from component frame to body frame
  libra::Matrix<kMtqDimension, kMtqDimension> scale_factor_;        //!< Scale factor matrix
  libra::Vector<kMtqDimension> max_magnetic_moment_c_Am2_{100.0};   //!< Maximum magnetic moment in the component frame [Am2]
  libra::Vector<kMtqDimension> min_magnetic_moment_c_Am2_{-100.0};  //!< Minimum magnetic moment in the component frame [Am2]

  libra::Vector<kMtqDimension> bias_noise_c_Am2_{0.0};   //!< Constant bias noise in the component frame [Am2]
  RandomWalk<kMtqDimension> random_walk_c_Am2_;          //!< Random walk noise
  libra::NormalRand random_noise_c_Am2_[kMtqDimension];  //!< Normal random noise

  const GeomagneticField* geomagnetic_field_;  //!< Geomagnetic environment

  /**
   * @fn CalcOutputTorque
   * @brief Calculate output torque
   * @return Output torque in the body fixed frame [Nm]
   */
  libra::Vector<kMtqDimension> CalcOutputTorque(void);
};

/**
 * @fn InitMagnetorquer
 * @brief Initialize functions for magnetometer without power port
 * @param [in] clock_generator: Clock generator
 * @param [in] actuator_id: Actuator ID
 * @param [in] file_name: Path to the initialize file
 * @param [in] component_step_time_s: Component step time [sec]
 * @param [in] geomagnetic_field: Geomegnetic environment
 */
Magnetorquer InitMagnetorquer(ClockGenerator* clock_generator, int actuator_id, const std::string file_name, double component_step_time_s,
                              const GeomagneticField* geomagnetic_field);
/**
 * @fn InitMagnetorquer
 * @brief Initialize functions for magnetometer with power port
 * @param [in] clock_generator: Clock generator
 * @param [in] power_port: Power port
 * @param [in] actuator_id: Actuator ID
 * @param [in] file_name: Path to the initialize file
 * @param [in] component_step_time_s: Component step time [sec]
 * @param [in] geomagnetic_field: Geomegnetic environment
 */
Magnetorquer InitMagnetorquer(ClockGenerator* clock_generator, PowerPort* power_port, int actuator_id, const std::string file_name,
                              double component_step_time_s, const GeomagneticField* geomagnetic_field);

#endif  // S2E_COMPONENTS_REAL_AOCS_MAGNETORQUER_HPP_
