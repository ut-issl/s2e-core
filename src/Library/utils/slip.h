#pragma once

// SLIP encoding

#include <stdint.h>
#include <vector>

std::vector<uint8_t> decode_slip(std::vector<uint8_t> in);
std::vector<uint8_t> encode_slip(std::vector<uint8_t> in);