/**
 * @file test_igs_product_name_handling.cpp
 * @brief Test functions for IGS product name handling with GoogleTest
 */
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "igs_product_name_handling.hpp"

using namespace s2e::gnss;

/**
 * @brief Test GetOrbitClockFinalFileName
 */
TEST(IgsProductName, GetOrbitClockFinalFileName) {
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

/**
 * @brief Test MergeYearDoy
 */
TEST(IgsProductName, MergeYearDoy) {
  size_t year = 2023;
  size_t doy = 190;
  size_t year_doy = MergeYearDoy(year, doy);

  EXPECT_EQ(2023190, year_doy);
}

/**
 * @brief Test PerseFromYearDoy
 */
TEST(IgsProductName, PerseFromYearDoy) {
  size_t year = 1989;
  size_t doy = 84;
  size_t year_doy = MergeYearDoy(year, doy);

  EXPECT_EQ(year, PerseYearFromYearDoy(year_doy));
  EXPECT_EQ(doy, PerseDoyFromYearDoy(year_doy));

  year_doy = 1989365;
  EXPECT_EQ(1989, PerseYearFromYearDoy(year_doy));
  EXPECT_EQ(365, PerseDoyFromYearDoy(year_doy));

  year_doy = 2000001;
  EXPECT_EQ(2000, PerseYearFromYearDoy(year_doy));
  EXPECT_EQ(001, PerseDoyFromYearDoy(year_doy));
}

/**
 * @brief Test IncrementYearDoy
 */
TEST(IgsProductName, IncrementYearDoy) {
  size_t year_doy = 2024030;
  year_doy = IncrementYearDoy(year_doy);
  EXPECT_EQ(2024031, year_doy);

  // Year update
  year_doy = 2023365;
  year_doy = IncrementYearDoy(year_doy);
  EXPECT_EQ(2024001, year_doy);

  // Leap year
  year_doy = 2024365;
  year_doy = IncrementYearDoy(year_doy);
  EXPECT_EQ(2024366, year_doy);
  year_doy = IncrementYearDoy(year_doy);
  EXPECT_EQ(2025001, year_doy);
}
