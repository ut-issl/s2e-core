#include <gtest/gtest.h>

#include "epoch_time.hpp"
#include "date_time_format.hpp"

TEST(EpochTime, ConstructorNominal) {
  EpochTime time(1000000000, 0.250);

  EXPECT_EQ(1000000000, time.GetTime_s());
  EXPECT_DOUBLE_EQ(0.250, time.GetFraction_s());
}

TEST(EpochTime, ConstructorWithCalender) {
  // Reference for correctness check: https://www.epochconverter.com/
  DateTime date_time("2023/6/5 13:40:59.4");
  EpochTime time(date_time);

  EXPECT_EQ(1685972459, time.GetTime_s());
  EXPECT_NEAR(0.4, time.GetFraction_s(), 1e-10);
}

TEST(EpochTime, TimeAdd) {
  EpochTime time1(1686145305, 0.3);
  EpochTime time2(245, 0.25);
  EpochTime time4(245, 0.8);

  // Add
  EpochTime time_add1 = TimeAdd(time1, time2);
  EpochTime time_add2 = TimeAdd(time1, time4);
  EXPECT_EQ(1686145305 + 245, time_add1.GetTime_s());
  EXPECT_NEAR(0.3 + 0.25, time_add1.GetFraction_s(), 1e-10);
  EXPECT_EQ(1686145305 + 245 + 1, time_add2.GetTime_s());
  EXPECT_NEAR(0.1, time_add2.GetFraction_s(), 1e-10);
}

TEST(EpochTime, TimeSubtract) {
  EpochTime time1(1686145305, 0.3);
  EpochTime time2(1686145245, 0.25);
  EpochTime time3(1686145245, 0.4);

  // Subtract
  EpochTime time_sub1 = TimeSubtract(time1, time2);
  EpochTime time_sub2 = TimeSubtract(time1, time3);
  EXPECT_EQ(1686145305 - 1686145245, time_sub1.GetTime_s());
  EXPECT_NEAR(0.3 - 0.25, time_sub1.GetFraction_s(), 1e-10);
  EXPECT_EQ(1686145305 - 1686145245 - 1, time_sub2.GetTime_s());
  EXPECT_NEAR(0.9, time_sub2.GetFraction_s(), 1e-10);
}
