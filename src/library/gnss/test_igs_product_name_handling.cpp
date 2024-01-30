/**
 * @file test_igs_product_name_handling.cpp
 * @brief Test functions for IGS product name handling with GoogleTest
 */
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "igs_product_name_handling.hpp"

/**
 * @brief Test satellite number to index
 */
TEST(IgsProductName, SatelliteNumberToIndex) {
  std::string header = "IGS0OPSFIN";
  size_t year_doy = 2023190;

  std::string file_name = GetOrbitClockFinalFileName(header, year_doy);
  EXPECT_THAT(file_name, ::testing::MatchesRegex("IGS0OPSFIN_20231900000_01D_15M_ORB.SP3"));

  std::string period = "15M";
  std::string file_type = "ORB.SP3";

  file_name = GetOrbitClockFinalFileName(header, year_doy, period, file_type);
  EXPECT_THAT(file_name, ::testing::MatchesRegex("IGS0OPSFIN_20231900000_01D_15M_ORB.SP3"));

  header = "IGS0OPSFIN";
  year_doy = 2023365;
  period = "30S";
  file_type = "CLK.CLK";

  file_name = GetOrbitClockFinalFileName(header, year_doy, period, file_type);
  EXPECT_THAT(file_name, ::testing::MatchesRegex("IGS0OPSFIN_20233650000_01D_30S_CLK.CLK"));
}