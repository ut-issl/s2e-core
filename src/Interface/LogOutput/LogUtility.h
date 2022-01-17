#pragma once

#include <iomanip>
#include <sstream>
#include <string>

#include "../../Library/math/MatVec.hpp"
#include "../../Library/math/Quaternion.hpp"
using libra::Matrix;
using libra::Quaternion;
using libra::Vector;

template <typename T>
inline std::string WriteScalar(T scalar, int precision = 6);
inline std::string WriteScalar(std::string name, std::string unit);

template <size_t NUM>
inline std::string WriteVector(Vector<NUM, double> vec, int precision = 6);
inline std::string WriteVector(std::string name, std::string frame,
                               std::string unit, int n);

template <size_t ROW, size_t COLUMN>
inline std::string WriteMatrix(Matrix<ROW, COLUMN, double> mat);
inline std::string WriteMatrix(std::string name, std::string frame,
                               std::string unit, int r, int c);

inline std::string WriteQuaternion(Quaternion quat);

// Libraries for log writing
template <typename T> std::string WriteScalar(T scalar, int precision) {
  std::stringstream str_tmp;
  str_tmp << std::setprecision(precision) << scalar << ",";
  return str_tmp.str();
}
std::string WriteScalar(std::string name, std::string unit) {
  return name + "[" + unit + "],";
}

template <size_t NUM>
std::string WriteVector(Vector<NUM, double> vec, int precision) {
  std::stringstream str_tmp;

  for (int n = 0; n < NUM; n++) {
    str_tmp << std::setprecision(precision) << vec[n] << ",";
  }
  return str_tmp.str();
}
std::string WriteVector(std::string name, std::string frame, std::string unit,
                        int n) {
  std::stringstream str_tmp;
  std::string axis[3] = {"(X)", "(Y)", "(Z)"};

  for (int i = 0; i < n; i++) {
    if (n == 3) {
      str_tmp << name << "_" << frame << axis[i] << "[" << unit << "],";
    } else {
      str_tmp << name << "_" << frame << "(" << i << ")"
              << "[" << unit << "],";
    }
  }
  return str_tmp.str();
}

template <size_t ROW, size_t COLUMN>
std::string WriteMatrix(Matrix<ROW, COLUMN, double> mat) {
  std::stringstream str_tmp;

  for (int n = 0; n < ROW; n++) {
    for (int m = 0; m < COLUMN; m++) {
      str_tmp << mat[n][m] << ",";
    }
  }
  return str_tmp.str();
}
std::string WriteMatrix(std::string name, std::string frame, std::string unit,
                        int r, int c) {
  std::stringstream str_tmp;

  for (int i = 0; i < r; i++) {
    for (int j = 0; j < c; j++) {
      str_tmp << name << "_" << frame << "(" << i << j << ")"
              << "[" << unit << "],";
    }
  }
  return str_tmp.str();
}

std::string WriteQuaternion(Quaternion quat) {
  std::stringstream str_tmp;

  for (int i = 0; i < 4; i++) {
    str_tmp << quat[i] << ",";
  }
  return str_tmp.str();
}
