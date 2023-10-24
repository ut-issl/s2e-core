#include <gtest/gtest.h>

#include "date_time_format.hpp"

/**
 * @brief Test Constructor with value
 */
TEST(DateTime, ConstructorNominal) {
  DateTime date_time(1989, 3, 25, 30, 50, 4.200);

  EXPECT_EQ(1989, date_time.GetYear());
  EXPECT_EQ(3, date_time.GetMonth());
  EXPECT_EQ(25, date_time.GetDay());
  EXPECT_EQ(30, date_time.GetHour());
  EXPECT_EQ(50, date_time.GetMinute());
  EXPECT_DOUBLE_EQ(4.200, date_time.GetSecond());
}

/**
 * @brief Test Constructor with string
 */
TEST(DateTime, ConstructorWithString) {
  DateTime date_time("2023/6/5 13:40:59.4");

  EXPECT_EQ(2023, date_time.GetYear());
  EXPECT_EQ(6, date_time.GetMonth());
  EXPECT_EQ(5, date_time.GetDay());
  EXPECT_EQ(13, date_time.GetHour());
  EXPECT_EQ(40, date_time.GetMinute());
  EXPECT_DOUBLE_EQ(59.4, date_time.GetSecond());
}

/**
 * @brief Test get as string
 */
TEST(DateTime, GetAsString) {
  DateTime date_time(1989, 3, 25, 30, 50, 4.200);
  DateTime date_time2(date_time.GetAsString());

  EXPECT_EQ(1989, date_time.GetYear());
  EXPECT_EQ(3, date_time.GetMonth());
  EXPECT_EQ(25, date_time.GetDay());
  EXPECT_EQ(30, date_time.GetHour());
  EXPECT_EQ(50, date_time.GetMinute());
  EXPECT_DOUBLE_EQ(4.200, date_time.GetSecond());

  EXPECT_EQ(date_time.GetYear(), date_time2.GetYear());
  EXPECT_EQ(date_time.GetMonth(), date_time2.GetMonth());
  EXPECT_EQ(date_time.GetDay(), date_time2.GetDay());
  EXPECT_EQ(date_time.GetHour(), date_time2.GetHour());
  EXPECT_EQ(date_time.GetMinute(), date_time2.GetMinute());
  EXPECT_DOUBLE_EQ(date_time.GetSecond(), date_time2.GetSecond());
}
