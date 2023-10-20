/**
 * @file test_quaternion.cpp
 * @brief Test codes for Quaternion class with GoogleTest
 */
#include <gtest/gtest.h>

#include "sp3_file_reader.hpp"

/**
 * @brief Test for Matrix * Vector
 */
TEST(Sp3FileReader, Constructor) { Sp3FileReader sp3_file("../../src/library/gnss/example.sp3"); }
