#include "Orbit.h"
#include "../../Library/math/ODE.hpp"

using namespace libra;

class SimpleCircularOrbit : public Orbit, public ODE<6>
{
private:
  static const int N = 6; // 状態量の次元数
  //static gravconsttype whichconst;
  double mu;

public:
  SimpleCircularOrbit(double mu, double timestep, int wgs);
  ~SimpleCircularOrbit();

  virtual void RHS(double t,
    const Vector<N>& state,
    Vector<N>& rhs);

  virtual void Initialize(Vector<3> init_position, Vector<3> init_velocity, double current_jd, double init_time = 0);

  // 軌道伝播の計算する
  virtual void Propagate(double current_jd);

  virtual void AddPositionOffset(Vector<3> offset_i);

  virtual string GetLogHeader() const;
  virtual string GetLogValue() const;

private:

};
