#include <gtest/gtest.h>

#include "gps_time.hpp"

/**
 * @brief Test Constructor
 */
TEST(GpsTime, ConstructorNominal) {
  GpsTime gps_time(2285, 12345.6798);

  EXPECT_EQ(2285, gps_time.GetWeek());
  EXPECT_DOUBLE_EQ(12345.6798, gps_time.GetElapsedTimeFromWeek_s());

  // Reference for correctness check: https://www.epochconverter.com/
  EXPECT_EQ(1697945145, gps_time.GetEpochTime().GetTime_s());
  EXPECT_NEAR(0.6798, gps_time.GetEpochTime().GetFraction_s(), 1e-10);

  // Reference for correctness check: https://gnsscalc.com/
  EXPECT_EQ(2023, gps_time.GetDateTime().GetYear());
  EXPECT_EQ(10, gps_time.GetDateTime().GetMonth());
  EXPECT_EQ(22, gps_time.GetDateTime().GetDay());
  EXPECT_EQ(3, gps_time.GetDateTime().GetHour());
  EXPECT_EQ(25, gps_time.GetDateTime().GetMinute());
  EXPECT_NEAR(45.6798, gps_time.GetDateTime().GetSecond(), 1e-10);

  // Check leap seconds
  EXPECT_EQ(2023, gps_time.GetDateTimeAsUtc().GetYear());
  EXPECT_EQ(10, gps_time.GetDateTimeAsUtc().GetMonth());
  EXPECT_EQ(22, gps_time.GetDateTimeAsUtc().GetDay());
  EXPECT_EQ(3, gps_time.GetDateTimeAsUtc().GetHour());
  EXPECT_EQ(26, gps_time.GetDateTimeAsUtc().GetMinute());
  EXPECT_NEAR(3.6798, gps_time.GetDateTimeAsUtc().GetSecond(), 1e-10);
}

/**
 * @brief Test Constructor with epoch time
 */
TEST(GpsTime, ConstructorWithEpochTime) {
  EpochTime unix_time(1686145305, 0.3);
  GpsTime gps_time(unix_time);

  // Reference for correctness check: https://www.epochconverter.com/
  EXPECT_EQ(2023, gps_time.GetDateTime().GetYear());
  EXPECT_EQ(6, gps_time.GetDateTime().GetMonth());
  EXPECT_EQ(7, gps_time.GetDateTime().GetDay());
  EXPECT_EQ(13, gps_time.GetDateTime().GetHour());
  EXPECT_EQ(41, gps_time.GetDateTime().GetMinute());
  EXPECT_NEAR(45.3, gps_time.GetDateTime().GetSecond(), 1e-10);

  // Reference for correctness check: https://gnsscalc.com/
  EXPECT_EQ(2265, gps_time.GetWeek());
  EXPECT_NEAR(308505.3, gps_time.GetElapsedTimeFromWeek_s(), 1e-10);
}

/**
 * @brief Test Constructor with date time
 */
TEST(GpsTime, ConstructorWithDateTime) {
  DateTime date_time("2023/6/5 13:40:59.4");
  GpsTime gps_time(date_time);

  // Reference for correctness check: https://www.epochconverter.com/
  EXPECT_EQ(1685972459, gps_time.GetEpochTime().GetTime_s());
  EXPECT_NEAR(0.4, gps_time.GetEpochTime().GetFraction_s(), 1e-10);

  // Reference for correctness check: https://gnsscalc.com/
  EXPECT_EQ(2265, gps_time.GetWeek());
  EXPECT_NEAR(135659.4, gps_time.GetElapsedTimeFromWeek_s(), 1e-10);
}
