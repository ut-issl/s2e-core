/**
 * @file first_order_lag.hpp
 * @brief A class to emulate the first order lag
 */

#ifndef S2E_LIBRARY_CONTROL_UTILITIES_FIRST_ORDER_LAG_HPP_
#define S2E_LIBRARY_CONTROL_UTILITIES_FIRST_ORDER_LAG_HPP_

/**
 * @class FirstOderLag
 * @brief A class to emulate the first order lag
 */
class FirstOderLag {
 public:
  /**
   * @fn FirstOderLag
   * @brief Default constructor
   */
  FirstOderLag(const double time_step_s = 1.0, const double time_constant_s = 1.0, const double gain = 1.0)
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

#endif  // S2E_LIBRARY_CONTROL_UTILITIES_FIRST_ORDER_LAG_HPP_
