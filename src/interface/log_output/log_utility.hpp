/**
 * @file log_utility.hpp
 * @brief Utility functions to support logging for users
 */

#ifndef S2E_INTERFACE_LOG_OUTPUT_LOG_UTILITY_H_
#define S2E_INTERFACE_LOG_OUTPUT_LOG_UTILITY_H_

#include <Library/math/MatVec.hpp>
#include <Library/math/Quaternion.hpp>
#include <iomanip>
#include <sstream>
#include <string>
using libra::Matrix;
using libra::Quaternion;
using libra::Vector;

/**
 * @fn WriteScalar
 * @brief Write scalar value
 * @param [in] scalar: scalar value
 * @param [in] precision: precision for the value (number of digit)
 */
template <typename T>
inline std::string WriteScalar(T scalar, int precision = 6);
/**
 * @fn WriteScalar
 * @brief Write header for scalar value
 * @param [in] name: Name of the scalar value
 * @param [in] unit: Unit of the scalar value
 */
inline std::string WriteScalar(std::string name, std::string unit);

/**
 * @fn WriteVector
 * @brief Write Vector value
 * @param [in] vec: vector value
 * @param [in] precision: precision for the value (number of digit)
 */
template <size_t NUM>
inline std::string WriteVector(Vector<NUM, double> vec, int precision = 6);
/**
 * @fn WriteVector
 * @brief Write header for vector value
 * @param [in] name: Name of the vector value
 * @param [in] frame: Frame of the vector value
 * @param [in] unit: Unit of the vector value
 * @param [in] n: Number of elements
 */
inline std::string WriteVector(std::string name, std::string frame, std::string unit, size_t n);

/**
 * @fn WriteMatrix
 * @brief Write Matrix value
 * @param [in] mat: matrix value
 * @note TODO: add precision?
 */
template <size_t ROW, size_t COLUMN>
inline std::string WriteMatrix(Matrix<ROW, COLUMN, double> mat);
/**
 * @fn WriteMatrix
 * @brief Write header for matrix value
 * @param [in] name: Name of the matrix value
 * @param [in] frame: Frame of the matrix value
 * @param [in] unit: Unit of the matrix value
 * @param [in] r: Row length
 * @param [in] c: Column length
 */
inline std::string WriteMatrix(std::string name, std::string frame, std::string unit, size_t r, size_t c);

/**
 * @fn WriteQuaternion
 * @brief Write quaternion value
 * @param [in] quat: Quaternion
 * @note TODO: add function for headers?
 */
inline std::string WriteQuaternion(Quaternion quat);
/**
 * @fn WriteQuaternion
 * @brief Write header for quaternion
 * @param [in] name: Name of the value
 * @param [in] frame: Frame of the quaternion
 */
inline std::string WriteQuaternion(std::string name, std::string frame);

//
// Libraries for log writing
//
template <typename T>
std::string WriteScalar(T scalar, int precision) {
  std::stringstream str_tmp;
  str_tmp << std::setprecision(precision) << scalar << ",";
  return str_tmp.str();
}
std::string WriteScalar(std::string name, std::string unit) { return name + "[" + unit + "],"; }

template <size_t NUM>
std::string WriteVector(Vector<NUM, double> vec, int precision) {
  std::stringstream str_tmp;

  for (size_t n = 0; n < NUM; n++) {
    str_tmp << std::setprecision(precision) << vec[n] << ",";
  }
  return str_tmp.str();
}
std::string WriteVector(std::string name, std::string frame, std::string unit, size_t n) {
  std::stringstream str_tmp;
  std::string axis[3] = {"_x", "_y", "_z"};

  for (size_t i = 0; i < n; i++) {
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

  for (size_t n = 0; n < ROW; n++) {
    for (size_t m = 0; m < COLUMN; m++) {
      str_tmp << mat[n][m] << ",";
    }
  }
  return str_tmp.str();
}
std::string WriteMatrix(std::string name, std::string frame, std::string unit, size_t r, size_t c) {
  std::stringstream str_tmp;

  for (size_t i = 0; i < r; i++) {
    for (size_t j = 0; j < c; j++) {
      str_tmp << name << "_" << frame << "(" << i << j << ")"
              << "[" << unit << "],";
    }
  }
  return str_tmp.str();
}

std::string WriteQuaternion(Quaternion quat) {
  std::stringstream str_tmp;

  for (size_t i = 0; i < 4; i++) {
    str_tmp << quat[i] << ",";
  }
  return str_tmp.str();
}
std::string WriteQuaternion(std::string name, std::string frame) {
  std::stringstream str_tmp;
  std::string axis[4] = {"_x", "_y", "_z", "_w"};

  for (size_t i = 0; i < 4; i++) {
    str_tmp << name << "_" << frame << axis[i] << ",";
  }
  return str_tmp.str();
}

#endif  // S2E_INTERFACE_LOG_OUTPUT_LOG_UTILITY_H_
