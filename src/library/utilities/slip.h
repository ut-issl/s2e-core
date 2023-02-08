/**
 * @file slip.h
 * @brief Functions for SLIP(Serial Line Internet Protocol) encoding
 */

#ifndef S2E_LIBRARY_UTILITIES_SLIP_HPP_
#define S2E_LIBRARY_UTILITIES_SLIP_HPP_

#include <stdint.h>

#include <vector>

/**
 * @fn decode_slip
 * @brief Decode SLIP data
 * @param [in] in: Input data
 * @return Decoded data
 */
std::vector<uint8_t> decode_slip(std::vector<uint8_t> in);
/**
 * @fn decode_slip_with_header
 * @brief Decode SLIP data with Header
 * @param [in] in: Input data
 * @return Decoded data
 */
std::vector<uint8_t> decode_slip_with_header(std::vector<uint8_t> in);

/**
 * @fn encode_slip
 * @brief Encode SLIP data
 * @param [in] in: Input data
 * @return Encoded data
 */
std::vector<uint8_t> encode_slip(std::vector<uint8_t> in);
/**
 * @fn encode_slip_with_header
 * @brief Encode SLIP data
 * @param [in] in: Input data
 * @return Encoded data
 */
std::vector<uint8_t> encode_slip_with_header(std::vector<uint8_t> in);

#endif  // S2E_LIBRARY_UTILITIES_SLIP_HPP_
