/**
 * @file test_quaternion.cpp
 * @brief Test codes for Quaternion class with GoogleTest
 */
#include <gtest/gtest.h>

#include "sp3_file_reader.hpp"

/**
 * @brief Test for Matrix * Vector
 */
TEST(Sp3FileReader, Constructor) {
  Sp3FileReader sp3_file("../../src/library/gnss/example.sp3");

  // Test Header
  EXPECT_EQ(2, sp3_file.GetNumberOfEpoch());
  EXPECT_EQ(119, sp3_file.GetNumberOfSatellites());

  EXPECT_EQ(2013, sp3_file.GetStartEpochDateTime().GetYear());
  EXPECT_EQ(4, sp3_file.GetStartEpochDateTime().GetMonth());
  EXPECT_EQ(3, sp3_file.GetStartEpochDateTime().GetDay());
  EXPECT_EQ(12, sp3_file.GetStartEpochDateTime().GetHour());
  EXPECT_EQ(4, sp3_file.GetStartEpochDateTime().GetMinute());
  EXPECT_DOUBLE_EQ(1.23456789, sp3_file.GetStartEpochDateTime().GetSecond());

  EXPECT_EQ(1734, sp3_file.GetStartEpochGpsTime().GetWeek());
  EXPECT_DOUBLE_EQ(259200.0, sp3_file.GetStartEpochGpsTime().GetElapsedTimeFromWeek_s());
  
  // TODO: Add other test for header read


  // Test epoch
}
