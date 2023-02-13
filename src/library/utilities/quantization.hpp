/**
 * @file quantization.hpp
 * @brief Functions for quantization
 */

#ifndef S2E_LIBRARY_UTILITIES_QUANTIZATION_HPP_
#define S2E_LIBRARY_UTILITIES_QUANTIZATION_HPP_

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

#endif  // S2E_LIBRARY_UTILITIES_QUANTIZATION_HPP_
