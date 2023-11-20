#include <gtest/gtest.h>

#include "gps_time.hpp"

/**
 * @brief Test Constructor
 */
TEST(GpsTime, ConstructorNominal) {
  GpsTime gps_time(2285, 12345.6798);

  EXPECT_EQ(2285, gps_time.GetWeek());
  EXPECT_DOUBLE_EQ(12345.6798, gps_time.GetElapsedTimeFromWeek_s());
}

/**
 * @brief Test Constructor with epoch time
 */
TEST(GpsTime, ConstructorWithEpochTime) {
  EpochTime unix_time(1686145305, 0.3);
  GpsTime gps_time(unix_time);

  // Reference for correctness check: https://gnsscalc.com/
  EXPECT_EQ(2265, gps_time.GetWeek());
  EXPECT_NEAR(308505.3, gps_time.GetElapsedTimeFromWeek_s(), 1e-10);
}
