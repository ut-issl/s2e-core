/**
 * @file test_gnss_satellite_number.cpp
 * @brief Test functions for GNSS satellite number handling with GoogleTest
 */
#include <gtest/gtest.h>

#include "gnss_satellite_number.hpp"

using namespace s2e::gnss;

/**
 * @brief Test satellite number to index
 */
TEST(GnssSatelliteNumber, SatelliteNumberToIndex) {
  EXPECT_EQ(ConvertGnssSatelliteNumberToIndex("G01"), 0);
  EXPECT_EQ(ConvertGnssSatelliteNumberToIndex("R02"), kGlonassIndexBegin + 1);
  EXPECT_EQ(ConvertGnssSatelliteNumberToIndex("E10"), kGalileoIndexBegin + 9);
  EXPECT_EQ(ConvertGnssSatelliteNumberToIndex("C40"), kBeidouIndexBegin + 39);
  EXPECT_EQ(ConvertGnssSatelliteNumberToIndex("J03"), kQzssIndexBegin + 2);
  EXPECT_EQ(ConvertGnssSatelliteNumberToIndex("I04"), kNavicIndexBegin + 3);
  EXPECT_EQ(ConvertGnssSatelliteNumberToIndex("err"), UINT32_MAX);
}

/**
 * @brief Test index to satellite number
 */
TEST(GnssSatelliteNumber, IndexToSatelliteNumber) {
  EXPECT_EQ(ConvertIndexToGnssSatelliteNumber(0), "G01");
  EXPECT_EQ(ConvertIndexToGnssSatelliteNumber(kGlonassIndexBegin + 9), "R10");
  EXPECT_EQ(ConvertIndexToGnssSatelliteNumber(kGalileoIndexBegin + 21), "E22");
  EXPECT_EQ(ConvertIndexToGnssSatelliteNumber(kBeidouIndexBegin + 50), "C51");
  EXPECT_EQ(ConvertIndexToGnssSatelliteNumber(kQzssIndexBegin + 0), "J01");
  EXPECT_EQ(ConvertIndexToGnssSatelliteNumber(kNavicIndexBegin + 5), "I06");
  EXPECT_EQ(ConvertIndexToGnssSatelliteNumber(5000), "err");
}
