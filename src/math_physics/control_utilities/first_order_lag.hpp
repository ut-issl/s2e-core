/**
 * @file first_order_lag.hpp
 * @brief A class to emulate the first order lag
 */

#ifndef S2E_LIBRARY_CONTROL_UTILITIES_FIRST_ORDER_LAG_HPP_
#define S2E_LIBRARY_CONTROL_UTILITIES_FIRST_ORDER_LAG_HPP_

namespace s2e::control_utilities {

/**
 * @class FirstOderLag
 * @brief A class to emulate the first order lag
 */
class FirstOrderLag {
 public:
  /**
   * @fn FirstOderLag
   * @brief Default constructor
   */
  FirstOrderLag(const double time_step_s = 1.0, const double time_constant_s = 1.0, const double gain = 1.0)
      : time_step_s_(time_step_s), time_constant_s_(time_constant_s), gain_(gain) {}

  /**
   * @fn Update
   * @brief Update State
   */
  inline double Update(const double input) {
    if (time_step_s_ + time_constant_s_ <= 0.0) return 0.0;
    const double c_out = time_constant_s_ / (time_step_s_ + time_constant_s_);
    const double c_in = (gain_ * time_step_s_) / (time_step_s_ + time_constant_s_);
    output_ = c_out * output_ + c_in * input;

    return output_;
  }

  /**
   * @fn GetOutput
   * @brief Return output
   */
  inline double GetOutput() const { return output_; }

 private:
  double output_ = 0.0;           //!< Output of the system
  const double time_step_s_;      //!< Time step [s]
  const double time_constant_s_;  //!< Time constant [s]
  const double gain_;             //!< Gain
};

}  // namespace s2e::control_utilities

#endif  // S2E_LIBRARY_CONTROL_UTILITIES_FIRST_ORDER_LAG_HPP_
