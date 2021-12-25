#pragma once

#include <string>
#include <sstream>
#include <iomanip>

using namespace std;

#include "../../Library/math/MatVec.hpp"
#include "../../Library/math/Quaternion.hpp"
using libra::Matrix;
using libra::Vector;
using libra::Quaternion;

template<typename T>
inline string WriteScalar(T scalar, int precision=6);
inline string WriteScalar(string name, string unit);

template<size_t NUM>
inline string WriteVector(Vector<NUM, double> vec, int precision=6);
inline string WriteVector(string name, string frame, string unit, int n);

template<size_t ROW, size_t COLUMN>
inline string WriteMatrix(Matrix<ROW, COLUMN, double> mat);
inline string WriteMatrix(string name, string frame, string unit, int r, int c);

inline string WriteQuaternion(Quaternion quat);

//Libraries for log writing
template<typename T>
string WriteScalar(T scalar, int precision)
{
  stringstream str_tmp;
  str_tmp << setprecision(precision) << scalar << ",";
  return str_tmp.str();
}
string WriteScalar(string name, string unit)
{
  return name + "[" + unit + "],";
}

template<size_t NUM>
string WriteVector(Vector<NUM, double> vec, int precision)
{
  stringstream str_tmp;

  for (int n = 0; n < NUM; n++)
  {
    str_tmp << setprecision(precision) << vec[n] << ",";
  }
  return str_tmp.str();
}
string WriteVector(string name, string frame, string unit, int n)
{
  stringstream str_tmp;
  string axis[3] = { "(X)", "(Y)", "(Z)" };


  for (int i = 0; i < n; i++)
  {
    if (n == 3)
    {
      str_tmp << name << "_" << frame << axis[i] << "[" << unit << "],";
    }
    else
    {
      str_tmp << name << "_" << frame << "(" << i << ")" << "[" << unit << "],";
    }
  }
  return str_tmp.str();
}

template<size_t ROW, size_t COLUMN>
string WriteMatrix(Matrix<ROW, COLUMN, double> mat)
{
  stringstream str_tmp;

  for (int n = 0; n < ROW; n++)
  {
    for (int m = 0; m < COLUMN; m++)
    {
      str_tmp << mat[n][m] << ",";
    }
  }
  return str_tmp.str();
}
string WriteMatrix(string name, string frame, string unit, int r, int c)
{
  stringstream str_tmp;

  for (int i = 0; i < r; i++)
  {
    for (int j = 0; j < c; j++)
    {
      str_tmp << name << "_" << frame << "(" << i << j << ")" << "[" << unit << "],";
    }
  }
  return str_tmp.str();
}

string WriteQuaternion(Quaternion quat)
{
  stringstream str_tmp;

  for (int i = 0; i < 4; i++)
  {
    str_tmp << quat[i] << ",";
  }
  return str_tmp.str();
}
