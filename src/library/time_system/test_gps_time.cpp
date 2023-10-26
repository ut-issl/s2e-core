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
