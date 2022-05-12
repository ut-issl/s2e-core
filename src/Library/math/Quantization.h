/*!
\file   Quantization.h
\author TAKISAWA Jun'ichi.
\date   Sat Oct  3 02:44:23 2009
\brief  量子化するための関数を格納．
*/
#ifndef QUANTIZATION_H_
#define QUANTIZATION_H_

//!入力
//! continuous_num : 連続する実数(double)
//! resolution : 解像度
//!出力
// quantized_num : 量子化された数(double)

double quantization(double continuous_num, double resolution);
float quantization_f(double continuous_num, double resolution);

#endif  // QUANTIZATION_H_
