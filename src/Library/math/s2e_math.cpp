#include "s2e_math.hpp"

namespace libra
{
double WrapTo2Pi(const double angle)
{
  double angle_out = angle;
  if (angle_out < 0.0)
  {
    while (angle_out < 0.0)
    {
      angle_out += 2.0*M_PI;
    }
  }
  else if (angle_out > 2.0*M_PI)
  {
    while (angle_out  > 2.0*M_PI)
    {
      angle_out -= 2.0*M_PI;
    }
  }
  else 
  {
    // nothing to do    
  }
  return angle_out;
}
}
