/*!
  \file   NormalRand_ifs.hpp
  \author TAKISAWA, Jun'ichi.
  \date   Sat May 14 00:25:14 2011
  \brief  NormalRandクラスのinline関数実装
*/
#ifndef NORMAL_RAND_IFS_HPP_
#define NORMAL_RAND_IFS_HPP_

namespace libra {

double NormalRand::avg() const { return avg_; }

void NormalRand::avg(double avg) { avg_ = avg; }

double NormalRand::stddev() const { return stddev_; }

void NormalRand::stddev(double stddev) { stddev_ = stddev; }

void NormalRand::set_param(double avg, double stddev) {
  avg_ = avg;
  stddev_ = stddev;
}

}  // namespace libra

#endif  // NORMAL_RAND_IFS_HPP_
