/**
 * @file sensor_base.hpp
 * @brief Base class for sensor emulation to add noises
 */

#ifndef S2E_COMPONENTS_BASE_CLASSES_SENSOR_BASE_H_
#define S2E_COMPONENTS_BASE_CLASSES_SENSOR_BASE_H_

#include <library/math/Matrix.hpp>
#include <library/math/NormalRand.hpp>
#include <library/math/RandomWalk.hpp>
#include <library/math/Vector.hpp>

/**
 * @class SensorBase
 * @brief Base class for sensor emulation to add noises
 * @note All sensors should inherit this class
 */
template <size_t N>
class SensorBase {
 public:
  /**
   * @fn SensorBase
   * @brief Constructor
   * @param [in] scale_factor: Scale factor matrix
   * @param [in] range_to_const_c: Output range limit to be constant output value at the component frame
   * @param [in] range_to_zero_c: Output range limit to be zero output value at the component frame
   * @param [in] bias_c: Constant bias noise at the component frame
   * @param [in] nr_stddev_c: Standard deviation of normal random noise at the component frame
   * @param [in] rw_stepwidth: Step width for random walk calculation [sec]
   * @param [in] rw_stddev_c: Standard deviation of random wark at the component frame
   * @param [in] rw_limit_c: Limit of random walk at the component frame
   */
  SensorBase(const libra::Matrix<N, N>& scale_factor, const libra::Vector<N>& range_to_const_c, const libra::Vector<N>& range_to_zero_c,
             const libra::Vector<N>& bias_c, const libra::Vector<N>& nr_stddev_c, double rw_stepwidth, const libra::Vector<N>& rw_stddev_c,
             const libra::Vector<N>& rw_limit_c);
  /**
   * @fn ~SensorBase
   * @brief Destructor
   */
  ~SensorBase();

 protected:
  /**
   * @fn Measure
   * @brief Return the observed data after adding the noise
   * @param [in] true_value_c: True value at the component frame
   * @return Observed value with noise at the component frame
   */
  libra::Vector<N> Measure(const libra::Vector<N> true_value_c);

 private:
  libra::Matrix<N, N> scale_factor_;   //!< Scale factor matrix
  libra::Vector<N> range_to_const_c_;  //!< Output range limit to be constant output value at the component frame
  libra::Vector<N> range_to_zero_c_;   //!< Output range limit to be zero output value at the component frame
  libra::Vector<N> bias_c_;            //!< Constant bias noise at the component frame
  libra::NormalRand nrs_c_[N];         //!< Normal random
  RandomWalk<N> n_rw_c_;               //!< Random Walk

  /**
   * @fn Clip
   * @brief Clipping according to the range information
   * @param [in] input_c: Input value at the component frame
   * @return Clipped value
   */
  libra::Vector<N> Clip(const libra::Vector<N> input_c);
  /**
   * @fn RangeCheck
   * @brief Check the range_to_const_c_ and range_to_zero_c_ is correct and fixed the values
   */
  void RangeCheck(void);
};

#include "./sensor_base_tfs.hpp"  // template function definisions.

#endif  // S2E_COMPONENTS_BASE_CLASSES_SENSOR_BASE_H_