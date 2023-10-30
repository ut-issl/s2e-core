/**
 * @file test_gnss_satellite_number.cpp
 * @brief Test functions for GNSS satellite number handling with GoogleTest
 */
#include <gtest/gtest.h>

#include "gnss_satellite_number.hpp"

/**
 * @brief Test satellite number to index
 */
TEST(GnssSatelliteNumber, SatelliteNumberToIndex) {
  EXPECT_EQ(ConvertSatelliteNumberToIndex("G01"), 0);
  EXPECT_EQ(ConvertSatelliteNumberToIndex("R02"), kGlonassIndexBegin + 1);
  EXPECT_EQ(ConvertSatelliteNumberToIndex("E10"), kGalileoIndexBegin + 9);
  EXPECT_EQ(ConvertSatelliteNumberToIndex("C40"), kBeidouIndexBegin + 39);
  EXPECT_EQ(ConvertSatelliteNumberToIndex("J03"), kQzssIndexBegin + 2);
  EXPECT_EQ(ConvertSatelliteNumberToIndex("I04"), kNavicIndexBegin + 3);
  EXPECT_EQ(ConvertSatelliteNumberToIndex("err"), UINT32_MAX);
}

/**
 * @brief Test index to satellite number
 */
TEST(GnssSatelliteNumber, IndexToSatelliteNumber) {
  EXPECT_EQ(ConvertIndexToSatelliteNumber(0), "G01");
  EXPECT_EQ(ConvertIndexToSatelliteNumber(kGlonassIndexBegin + 9), "R10");
  EXPECT_EQ(ConvertIndexToSatelliteNumber(kGalileoIndexBegin + 21), "E22");
  EXPECT_EQ(ConvertIndexToSatelliteNumber(kBeidouIndexBegin + 50), "C51");
  EXPECT_EQ(ConvertIndexToSatelliteNumber(kQzssIndexBegin + 0), "J01");
  EXPECT_EQ(ConvertIndexToSatelliteNumber(kNavicIndexBegin + 5), "I06");
  EXPECT_EQ(ConvertIndexToSatelliteNumber(5000), "err");
}
