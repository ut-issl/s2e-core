#ifndef LPF_H_
#define LPF_H_

class LPF {
 public:
  LPF(double omega_c, double sampling_t);
  double operator()(double u);

 private:
  double y_[3];
  double u_[3];

  double a_[3];
  double b_[3];
};

#endif  // LPF_H_
