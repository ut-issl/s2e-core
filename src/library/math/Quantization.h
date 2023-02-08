/**
 * @file Quantization.h
 * @brief Functions for quantization
 */

#ifndef QUANTIZATION_H_
#define QUANTIZATION_H_

/**
 * @fn quantization
 * @brief Default constructor without any initialization
 * @param [in] continuous_num: Target number
 * @param [in] resolution: Resolution of the quantization
 * @return Quantized value (double)
 */
double quantization(double continuous_num, double resolution);

/**
 * @fn quantization_f
 * @brief Default constructor without any initialization
 * @param [in] continuous_num: Target number
 * @param [in] resolution: Resolution of the quantization
 * @return Quantized value (float)
 */
float quantization_f(double continuous_num, double resolution);

#endif  // QUANTIZATION_H_
