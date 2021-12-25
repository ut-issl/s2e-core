#define _USE_MATH_DEFINES
#include <math.h>

#include "UWBSensor.h"

UWBSensor::UWBSensor(int sensor_id, Vector<3> pos_b_, Vector<3> dir_b_, Vector<3> axis_b_)
  :sensor_id(sensor_id), pos_b(pos_b_), dir_b(dir_b_), axis_b(axis_b_), measure_nr(0,1,sensor_id)
{
}

UWBSensor::~UWBSensor()
{
}

int UWBSensor::IsVisible(UWBSensor & other)
{
  UWBSensor& o = other;
  // チェーサーUWBからターゲットUWBへの相対位置ベクトル(慣性座標系)
  Vector<3> rel_pos = LocationTo(other);

  Vector<3> dir_i = q_b2i.frame_conv(dir_b);
  Vector<3> o_dir_i = o.q_b2i.frame_conv(o.dir_b);

  // UWB方向ベクトル間の内積(cos)
  double dot_uwbs = dot(dir_i, o_dir_i);

  // UWB間相対位置ベクトルと受信側UWB方向ベクトル間の内積
  double dot_sigdir = dot(rel_pos, dir_i);
  if (dot_uwbs > 0 || dot_sigdir < 0)
  {
    // UWBのアンテナが非可視の位置関係にある
    return 0;
  }

  double theta = angle(q_b2i.frame_conv(axis_b), rel_pos);
  double o_theta = angle(o.q_b2i.frame_conv(o.axis_b), rel_pos);

  double propagate_loss = -20 * log10(4 * M_PI*fc*norm(rel_pos) / c);
  double received_gain = Pt + propagate_loss + CalcAntennaGain(theta) + o.CalcAntennaGain(o_theta);

  if (received_gain > Plimit) return 1;
  
  return 0;
}

Vector<3> UWBSensor::LocationTo(UWBSensor & other)
{
  UWBSensor& o = other;
  // チェーサーUWBからターゲットUWBへの相対位置ベクトル(慣性座標系)
  Vector<3> rel_pos = o.ref_pos_i - ref_pos_i - q_b2i.frame_conv(pos_b) + o.q_b2i.frame_conv(o.pos_b);
  return rel_pos;
}

double UWBSensor::MeasureDistanceTo(UWBSensor & other)
{
  auto rel_pos = LocationTo(other);
  double distance = norm(rel_pos);
  double noise = measure_nr * CalcDeviation(distance);
  return distance + noise;
}

void UWBSensor::SetParameters(Vector<3> pos_i, Quaternion q_i2b)
{
  ref_pos_i = pos_i;
  q_b2i = q_i2b.conjugate();
}

double UWBSensor::CalcAntennaGain(double theta)
{
  return 2.15 + 20 * log10(cos(M_PI/2*cos(theta))/sin(theta));
}

double UWBSensor::CalcDeviation(double distance)
{
  if (distance < 200 && distance >= 150)
    return 0.07;
  if (distance < 150 && distance >= 100)
    return 0.05;
  if (distance < 100)
    return 0.03;
  return 0.1;
}
