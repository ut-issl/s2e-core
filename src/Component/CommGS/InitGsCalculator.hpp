/*
 * @file InitGscalculator.hpp
 * @brief Initialize function for Ground Station Calculator
 */

#pragma once

#include <Component/CommGS/GScalculator.h>

/*
 * @fn InitGscalculator
 * @brief Initialize function for Ground Station Calculator
 * @param [in] fname: Path to initialize file
 */

GScalculator InitGScalculator(const std::string fname);
