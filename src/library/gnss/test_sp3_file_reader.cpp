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
  std::string test_file_name = "/src/library/gnss/example.sp3";
  Sp3FileReader sp3_file(CORE_DIR_FROM_EXE + test_file_name);

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
  EXPECT_EQ(2013, sp3_file.GetEpochData(0).GetYear());
  EXPECT_EQ(4, sp3_file.GetEpochData(0).GetMonth());
  EXPECT_EQ(3, sp3_file.GetEpochData(0).GetDay());
  EXPECT_EQ(12, sp3_file.GetEpochData(0).GetHour());
  EXPECT_EQ(4, sp3_file.GetEpochData(0).GetMinute());
  EXPECT_DOUBLE_EQ(1.23456789, sp3_file.GetEpochData(0).GetSecond());

  EXPECT_EQ(2013, sp3_file.GetEpochData(1).GetYear());
  EXPECT_EQ(4, sp3_file.GetEpochData(1).GetMonth());
  EXPECT_EQ(3, sp3_file.GetEpochData(1).GetDay());
  EXPECT_EQ(12, sp3_file.GetEpochData(1).GetHour());
  EXPECT_EQ(19, sp3_file.GetEpochData(1).GetMinute());
  EXPECT_DOUBLE_EQ(1.23456789, sp3_file.GetEpochData(1).GetSecond());

  // Test Orbit
  // TODO: Test all satellite?
  // First epoch, First satellite
  EXPECT_DOUBLE_EQ(-15163.034377, sp3_file.GetSatellitePosition_km(0, 0)[0]);
  EXPECT_DOUBLE_EQ(-1301.934894, sp3_file.GetSatellitePosition_km(0, 0)[1]);
  EXPECT_DOUBLE_EQ(21473.065529, sp3_file.GetSatellitePosition_km(0, 0)[2]);
  EXPECT_DOUBLE_EQ(171.736636, sp3_file.GetSatelliteClockOffset(0, 0));
  // Last epoch, Last satellite
  EXPECT_DOUBLE_EQ(-32428.005614, sp3_file.GetSatellitePosition_km(1, 118)[0]);
  EXPECT_DOUBLE_EQ(24629.027242, sp3_file.GetSatellitePosition_km(1, 118)[1]);
  EXPECT_DOUBLE_EQ(-5590.944265, sp3_file.GetSatellitePosition_km(1, 118)[2]);
  EXPECT_DOUBLE_EQ(97.367506, sp3_file.GetSatelliteClockOffset(1, 118));
}
