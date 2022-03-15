#ifndef MTQ_H_
#define MTQ_H_

#include <Environment/Local/LocalEnvironment.h>
#include <Interface/LogOutput/ILoggable.h>

#include <Library/math/Matrix.hpp>
#include <Library/math/NormalRand.hpp>
#include <Library/math/Quaternion.hpp>
#include <Library/math/RandomWalk.hpp>
#include <Library/math/Vector.hpp>

#include "../Abstract/ComponentBase.h"

const size_t kMtqDim = 3;

class MagTorquer : public ComponentBase, public ILoggable {
 public:
  //! Constructor
  /*!
  \prescaler, clock_gen : For ComponentBase
  \id : ID for log output
  \q_b2c : Quarternion from body frame to component frame
  \max_c : Maximum magnetic moment [Am2]
  \min_c : Minimum magnetic moment [Am2]
  \bias_c : Constant bias noise [Am2]
  \rw_stepwidth : Step width for Random Walk [s]
  \rw_stddev: Standard deviation for Random Walk noise [Am2]
  \rw_limit: Limit for  Random Walk noise [Am2]
  \nr_stddev: Standard deviation for the normal random noise [Am2]
  \resolution: Output resolution [Am2]
  */
  MagTorquer(const int prescaler, ClockGenerator* clock_gen, const int id,
             const libra::Quaternion& q_b2c,
             const libra::Matrix<kMtqDim, kMtqDim>& scale_facter,
             const libra::Vector<kMtqDim>& max_c,
             const libra::Vector<kMtqDim>& min_c,
             const libra::Vector<kMtqDim>& bias_c, double rw_stepwidth,
             const libra::Vector<kMtqDim>& rw_stddev_c,
             const libra::Vector<kMtqDim>& rw_limit_c,
             const libra::Vector<kMtqDim>& nr_stddev_c,
             const MagEnvironment* mag_env);
  MagTorquer(const int prescaler, ClockGenerator* clock_gen,
             PowerPort* power_port, const int id,
             const libra::Quaternion& q_b2c,
             const libra::Matrix<kMtqDim, kMtqDim>& scale_facter,
             const libra::Vector<kMtqDim>& max_c,
             const libra::Vector<kMtqDim>& min_c,
             const libra::Vector<kMtqDim>& bias_c, double rw_stepwidth,
             const libra::Vector<kMtqDim>& rw_stddev_c,
             const libra::Vector<kMtqDim>& rw_limit_c,
             const libra::Vector<kMtqDim>& nr_stddev_c,
             const MagEnvironment* mag_env);

  // ComponentBase override functions
  void MainRoutine(int count) override;
  // ILogabble override functions
  virtual std::string GetLogHeader() const;
  virtual std::string GetLogValue() const;

  // Getter
  inline const libra::Vector<kMtqDim>& GetMagMoment_b(void) const {
    return mag_moment_b_;
  };  // Am2
  inline const libra::Vector<kMtqDim>& GetTorque_b(void) const {
    return torque_b_;
  };  // Nm
  // Setter
  inline void SetMagMomentC(const libra::Vector<kMtqDim> mag_moment_c) {
    mag_moment_c_ = mag_moment_c;
  };  // Am2

 protected:
  const int id_ = 0;
  const double knT2T = 1.0e-9;
  libra::Vector<kMtqDim> torque_b_{0.0};      // Nm
  libra::Vector<kMtqDim> mag_moment_c_{0.0};  // Am2
  libra::Vector<kMtqDim> mag_moment_b_{0.0};  // Am2

  libra::Quaternion q_b2c_{
      0.0, 0.0, 0.0, 1.0};  // Quarternion from body frame to component frame
  libra::Quaternion q_c2b_{
      0.0, 0.0, 0.0, 1.0};  // Quarternion from component frame to body frame
  libra::Matrix<kMtqDim, kMtqDim> scale_factor_;
  libra::Vector<kMtqDim> max_c_{100.0};   // Am2
  libra::Vector<kMtqDim> min_c_{-100.0};  // Am2
  libra::Vector<kMtqDim> bias_c_{0.0};    // Am2;
  //! Random Walk
  RandomWalk<kMtqDim> n_rw_c_;
  //! Normal random noise
  libra::NormalRand nrs_c_[kMtqDim];  // Am2

  const MagEnvironment* mag_env_;
  libra::Vector<kMtqDim> CalcOutputTorque(void);
};

#endif  // MTQ_H_
