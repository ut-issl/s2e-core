/**
 * @file test_antex_file_reader.cpp
 * @brief Test codes for AntexReader class with GoogleTest
 */
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "antex_file_reader.hpp"

/**
 * @brief Test Constructor
 */
TEST(AntexReader, Constructor) {
  std::string test_file_name = "/src/library/gnss/example.atx";

  AntexFileReader antex_file_fault("false_file_path.atx");
  EXPECT_FALSE(antex_file_fault.GetFileReadSuccessFlag());

  AntexFileReader antex_file(CORE_DIR_FROM_EXE + test_file_name);
  EXPECT_TRUE(antex_file.GetFileReadSuccessFlag());

  // Check data
  EXPECT_EQ(59, antex_file.GetNumberOfSatelliteData());

  // Check first data
  AntexSatelliteData antex_satellite_data = antex_file.GetAntexSatelliteData(0)[0];
  EXPECT_THAT(antex_satellite_data.GetAntennaType(), ::testing::MatchesRegex("BLOCK IIA.*"));
  EXPECT_THAT(antex_satellite_data.GetSerialNumber(), ::testing::MatchesRegex("G01.*"));
  EXPECT_EQ(2, antex_satellite_data.GetNumberOfFrequency());
  EXPECT_THAT(antex_satellite_data.GetValidStartTime().GetAsString(), ::testing::MatchesRegex("1992/11/22 00:00:0.*"));
  EXPECT_THAT(antex_satellite_data.GetValidEndTime().GetAsString(), ::testing::MatchesRegex("2008/10/16 23:59:60.*"));
  // First frequency
  EXPECT_THAT(antex_satellite_data.GetPhaseCenterData(0).GetFrequencyName(), ::testing::MatchesRegex("G01.*"));
  EXPECT_DOUBLE_EQ(0.0, antex_satellite_data.GetPhaseCenterData(0).GetGridInformation().GetZenithStartAngle_deg());
  EXPECT_DOUBLE_EQ(17.0, antex_satellite_data.GetPhaseCenterData(0).GetGridInformation().GetZenithEndAngle_deg());
  EXPECT_DOUBLE_EQ(1.0, antex_satellite_data.GetPhaseCenterData(0).GetGridInformation().GetZenithStepAngle_deg());
  EXPECT_EQ(18, antex_satellite_data.GetPhaseCenterData(0).GetGridInformation().GetNumberOfZenithGrid());
  EXPECT_DOUBLE_EQ(279.0, antex_satellite_data.GetPhaseCenterData(0).GetPhaseCenterOffset_mm()[0]);
  EXPECT_DOUBLE_EQ(0.0, antex_satellite_data.GetPhaseCenterData(0).GetPhaseCenterOffset_mm()[1]);
  EXPECT_DOUBLE_EQ(2319.5, antex_satellite_data.GetPhaseCenterData(0).GetPhaseCenterOffset_mm()[2]);
  EXPECT_DOUBLE_EQ(-0.8, antex_satellite_data.GetPhaseCenterData(0).GetPhaseCenterVariationParameters_mm()[0]);
  EXPECT_DOUBLE_EQ(-0.9, antex_satellite_data.GetPhaseCenterData(0).GetPhaseCenterVariationParameters_mm()[1]);
  EXPECT_DOUBLE_EQ(-0.9, antex_satellite_data.GetPhaseCenterData(0).GetPhaseCenterVariationParameters_mm()[2]);
  EXPECT_DOUBLE_EQ(-0.8, antex_satellite_data.GetPhaseCenterData(0).GetPhaseCenterVariationParameters_mm()[3]);
  EXPECT_DOUBLE_EQ(-0.4, antex_satellite_data.GetPhaseCenterData(0).GetPhaseCenterVariationParameters_mm()[4]);
  EXPECT_DOUBLE_EQ(0.2, antex_satellite_data.GetPhaseCenterData(0).GetPhaseCenterVariationParameters_mm()[5]);
  EXPECT_DOUBLE_EQ(0.8, antex_satellite_data.GetPhaseCenterData(0).GetPhaseCenterVariationParameters_mm()[6]);
  EXPECT_DOUBLE_EQ(1.3, antex_satellite_data.GetPhaseCenterData(0).GetPhaseCenterVariationParameters_mm()[7]);
  EXPECT_DOUBLE_EQ(1.4, antex_satellite_data.GetPhaseCenterData(0).GetPhaseCenterVariationParameters_mm()[8]);
  EXPECT_DOUBLE_EQ(1.2, antex_satellite_data.GetPhaseCenterData(0).GetPhaseCenterVariationParameters_mm()[9]);
  EXPECT_DOUBLE_EQ(0.7, antex_satellite_data.GetPhaseCenterData(0).GetPhaseCenterVariationParameters_mm()[10]);
  EXPECT_DOUBLE_EQ(0.0, antex_satellite_data.GetPhaseCenterData(0).GetPhaseCenterVariationParameters_mm()[11]);
  EXPECT_DOUBLE_EQ(-0.4, antex_satellite_data.GetPhaseCenterData(0).GetPhaseCenterVariationParameters_mm()[12]);
  EXPECT_DOUBLE_EQ(-0.7, antex_satellite_data.GetPhaseCenterData(0).GetPhaseCenterVariationParameters_mm()[13]);
  EXPECT_DOUBLE_EQ(-0.9, antex_satellite_data.GetPhaseCenterData(0).GetPhaseCenterVariationParameters_mm()[14]);
  EXPECT_DOUBLE_EQ(-0.9, antex_satellite_data.GetPhaseCenterData(0).GetPhaseCenterVariationParameters_mm()[15]);
  EXPECT_DOUBLE_EQ(-0.9, antex_satellite_data.GetPhaseCenterData(0).GetPhaseCenterVariationParameters_mm()[16]);
  EXPECT_DOUBLE_EQ(-0.9, antex_satellite_data.GetPhaseCenterData(0).GetPhaseCenterVariationParameters_mm()[17]);
  // Second frequency
  EXPECT_THAT(antex_satellite_data.GetPhaseCenterData(1).GetFrequencyName(), ::testing::MatchesRegex("G02.*"));
  EXPECT_DOUBLE_EQ(0.0, antex_satellite_data.GetPhaseCenterData(1).GetGridInformation().GetZenithStartAngle_deg());
  EXPECT_DOUBLE_EQ(17.0, antex_satellite_data.GetPhaseCenterData(1).GetGridInformation().GetZenithEndAngle_deg());
  EXPECT_DOUBLE_EQ(1.0, antex_satellite_data.GetPhaseCenterData(1).GetGridInformation().GetZenithStepAngle_deg());
  EXPECT_EQ(18, antex_satellite_data.GetPhaseCenterData(1).GetGridInformation().GetNumberOfZenithGrid());
  EXPECT_DOUBLE_EQ(279.0, antex_satellite_data.GetPhaseCenterData(1).GetPhaseCenterOffset_mm()[0]);
  EXPECT_DOUBLE_EQ(0.0, antex_satellite_data.GetPhaseCenterData(1).GetPhaseCenterOffset_mm()[1]);
  EXPECT_DOUBLE_EQ(2319.5, antex_satellite_data.GetPhaseCenterData(1).GetPhaseCenterOffset_mm()[2]);
  EXPECT_DOUBLE_EQ(-0.8, antex_satellite_data.GetPhaseCenterData(1).GetPhaseCenterVariationParameters_mm()[0]);
  EXPECT_DOUBLE_EQ(-0.9, antex_satellite_data.GetPhaseCenterData(1).GetPhaseCenterVariationParameters_mm()[1]);
  EXPECT_DOUBLE_EQ(-0.9, antex_satellite_data.GetPhaseCenterData(1).GetPhaseCenterVariationParameters_mm()[2]);
  EXPECT_DOUBLE_EQ(-0.8, antex_satellite_data.GetPhaseCenterData(1).GetPhaseCenterVariationParameters_mm()[3]);
  EXPECT_DOUBLE_EQ(-0.4, antex_satellite_data.GetPhaseCenterData(1).GetPhaseCenterVariationParameters_mm()[4]);
  EXPECT_DOUBLE_EQ(0.2, antex_satellite_data.GetPhaseCenterData(1).GetPhaseCenterVariationParameters_mm()[5]);
  EXPECT_DOUBLE_EQ(0.8, antex_satellite_data.GetPhaseCenterData(1).GetPhaseCenterVariationParameters_mm()[6]);
  EXPECT_DOUBLE_EQ(1.3, antex_satellite_data.GetPhaseCenterData(1).GetPhaseCenterVariationParameters_mm()[7]);
  EXPECT_DOUBLE_EQ(1.4, antex_satellite_data.GetPhaseCenterData(1).GetPhaseCenterVariationParameters_mm()[8]);
  EXPECT_DOUBLE_EQ(1.2, antex_satellite_data.GetPhaseCenterData(1).GetPhaseCenterVariationParameters_mm()[9]);
  EXPECT_DOUBLE_EQ(0.7, antex_satellite_data.GetPhaseCenterData(1).GetPhaseCenterVariationParameters_mm()[10]);
  EXPECT_DOUBLE_EQ(0.0, antex_satellite_data.GetPhaseCenterData(1).GetPhaseCenterVariationParameters_mm()[11]);
  EXPECT_DOUBLE_EQ(-0.4, antex_satellite_data.GetPhaseCenterData(1).GetPhaseCenterVariationParameters_mm()[12]);
  EXPECT_DOUBLE_EQ(-0.7, antex_satellite_data.GetPhaseCenterData(1).GetPhaseCenterVariationParameters_mm()[13]);
  EXPECT_DOUBLE_EQ(-0.9, antex_satellite_data.GetPhaseCenterData(1).GetPhaseCenterVariationParameters_mm()[14]);
  EXPECT_DOUBLE_EQ(-0.9, antex_satellite_data.GetPhaseCenterData(1).GetPhaseCenterVariationParameters_mm()[15]);
  EXPECT_DOUBLE_EQ(-0.9, antex_satellite_data.GetPhaseCenterData(1).GetPhaseCenterVariationParameters_mm()[16]);
  EXPECT_DOUBLE_EQ(-0.9, antex_satellite_data.GetPhaseCenterData(1).GetPhaseCenterVariationParameters_mm()[17]);

  // Check last data
  // Check first data
  antex_satellite_data = antex_file.GetAntexSatelliteData(58)[1];
  EXPECT_THAT(antex_satellite_data.GetAntennaType(), ::testing::MatchesRegex("GLONASS-M.*"));
  EXPECT_THAT(antex_satellite_data.GetSerialNumber(), ::testing::MatchesRegex("R27.*"));
  EXPECT_EQ(2, antex_satellite_data.GetNumberOfFrequency());
  EXPECT_THAT(antex_satellite_data.GetValidStartTime().GetAsString(), ::testing::MatchesRegex("2019/08/03 00:00:0.*"));
  // First frequency
  EXPECT_THAT(antex_satellite_data.GetPhaseCenterData(0).GetFrequencyName(), ::testing::MatchesRegex("R01.*"));
  EXPECT_DOUBLE_EQ(0.0, antex_satellite_data.GetPhaseCenterData(0).GetGridInformation().GetZenithStartAngle_deg());
  EXPECT_DOUBLE_EQ(15.0, antex_satellite_data.GetPhaseCenterData(0).GetGridInformation().GetZenithEndAngle_deg());
  EXPECT_DOUBLE_EQ(1.0, antex_satellite_data.GetPhaseCenterData(0).GetGridInformation().GetZenithStepAngle_deg());
  EXPECT_EQ(16, antex_satellite_data.GetPhaseCenterData(0).GetGridInformation().GetNumberOfZenithGrid());
  EXPECT_DOUBLE_EQ(-545.0, antex_satellite_data.GetPhaseCenterData(0).GetPhaseCenterOffset_mm()[0]);
  EXPECT_DOUBLE_EQ(0.0, antex_satellite_data.GetPhaseCenterData(0).GetPhaseCenterOffset_mm()[1]);
  EXPECT_DOUBLE_EQ(2451.2, antex_satellite_data.GetPhaseCenterData(0).GetPhaseCenterOffset_mm()[2]);
  EXPECT_DOUBLE_EQ(1.9, antex_satellite_data.GetPhaseCenterData(0).GetPhaseCenterVariationParameters_mm()[0]);
  EXPECT_DOUBLE_EQ(1.5, antex_satellite_data.GetPhaseCenterData(0).GetPhaseCenterVariationParameters_mm()[1]);
  EXPECT_DOUBLE_EQ(1.1, antex_satellite_data.GetPhaseCenterData(0).GetPhaseCenterVariationParameters_mm()[2]);
  EXPECT_DOUBLE_EQ(0.8, antex_satellite_data.GetPhaseCenterData(0).GetPhaseCenterVariationParameters_mm()[3]);
  EXPECT_DOUBLE_EQ(0.2, antex_satellite_data.GetPhaseCenterData(0).GetPhaseCenterVariationParameters_mm()[4]);
  EXPECT_DOUBLE_EQ(-0.2, antex_satellite_data.GetPhaseCenterData(0).GetPhaseCenterVariationParameters_mm()[5]);
  EXPECT_DOUBLE_EQ(-0.6, antex_satellite_data.GetPhaseCenterData(0).GetPhaseCenterVariationParameters_mm()[6]);
  EXPECT_DOUBLE_EQ(-1.1, antex_satellite_data.GetPhaseCenterData(0).GetPhaseCenterVariationParameters_mm()[7]);
  EXPECT_DOUBLE_EQ(-1.3, antex_satellite_data.GetPhaseCenterData(0).GetPhaseCenterVariationParameters_mm()[8]);
  EXPECT_DOUBLE_EQ(-1.6, antex_satellite_data.GetPhaseCenterData(0).GetPhaseCenterVariationParameters_mm()[9]);
  EXPECT_DOUBLE_EQ(-1.8, antex_satellite_data.GetPhaseCenterData(0).GetPhaseCenterVariationParameters_mm()[10]);
  EXPECT_DOUBLE_EQ(-1.6, antex_satellite_data.GetPhaseCenterData(0).GetPhaseCenterVariationParameters_mm()[11]);
  EXPECT_DOUBLE_EQ(-1.1, antex_satellite_data.GetPhaseCenterData(0).GetPhaseCenterVariationParameters_mm()[12]);
  EXPECT_DOUBLE_EQ(0.0, antex_satellite_data.GetPhaseCenterData(0).GetPhaseCenterVariationParameters_mm()[13]);
  EXPECT_DOUBLE_EQ(1.5, antex_satellite_data.GetPhaseCenterData(0).GetPhaseCenterVariationParameters_mm()[14]);
  EXPECT_DOUBLE_EQ(2.2, antex_satellite_data.GetPhaseCenterData(0).GetPhaseCenterVariationParameters_mm()[15]);
  // Second frequency
  EXPECT_THAT(antex_satellite_data.GetPhaseCenterData(1).GetFrequencyName(), ::testing::MatchesRegex("R02.*"));
  EXPECT_DOUBLE_EQ(0.0, antex_satellite_data.GetPhaseCenterData(1).GetGridInformation().GetZenithStartAngle_deg());
  EXPECT_DOUBLE_EQ(15.0, antex_satellite_data.GetPhaseCenterData(1).GetGridInformation().GetZenithEndAngle_deg());
  EXPECT_DOUBLE_EQ(1.0, antex_satellite_data.GetPhaseCenterData(1).GetGridInformation().GetZenithStepAngle_deg());
  EXPECT_EQ(16, antex_satellite_data.GetPhaseCenterData(1).GetGridInformation().GetNumberOfZenithGrid());
  EXPECT_DOUBLE_EQ(-545.0, antex_satellite_data.GetPhaseCenterData(1).GetPhaseCenterOffset_mm()[0]);
  EXPECT_DOUBLE_EQ(0.0, antex_satellite_data.GetPhaseCenterData(1).GetPhaseCenterOffset_mm()[1]);
  EXPECT_DOUBLE_EQ(2451.2, antex_satellite_data.GetPhaseCenterData(1).GetPhaseCenterOffset_mm()[2]);
  EXPECT_DOUBLE_EQ(1.9, antex_satellite_data.GetPhaseCenterData(1).GetPhaseCenterVariationParameters_mm()[0]);
  EXPECT_DOUBLE_EQ(1.5, antex_satellite_data.GetPhaseCenterData(1).GetPhaseCenterVariationParameters_mm()[1]);
  EXPECT_DOUBLE_EQ(1.1, antex_satellite_data.GetPhaseCenterData(1).GetPhaseCenterVariationParameters_mm()[2]);
  EXPECT_DOUBLE_EQ(0.8, antex_satellite_data.GetPhaseCenterData(1).GetPhaseCenterVariationParameters_mm()[3]);
  EXPECT_DOUBLE_EQ(0.2, antex_satellite_data.GetPhaseCenterData(1).GetPhaseCenterVariationParameters_mm()[4]);
  EXPECT_DOUBLE_EQ(-0.2, antex_satellite_data.GetPhaseCenterData(1).GetPhaseCenterVariationParameters_mm()[5]);
  EXPECT_DOUBLE_EQ(-0.6, antex_satellite_data.GetPhaseCenterData(1).GetPhaseCenterVariationParameters_mm()[6]);
  EXPECT_DOUBLE_EQ(-1.1, antex_satellite_data.GetPhaseCenterData(1).GetPhaseCenterVariationParameters_mm()[7]);
  EXPECT_DOUBLE_EQ(-1.3, antex_satellite_data.GetPhaseCenterData(1).GetPhaseCenterVariationParameters_mm()[8]);
  EXPECT_DOUBLE_EQ(-1.6, antex_satellite_data.GetPhaseCenterData(1).GetPhaseCenterVariationParameters_mm()[9]);
  EXPECT_DOUBLE_EQ(-1.8, antex_satellite_data.GetPhaseCenterData(1).GetPhaseCenterVariationParameters_mm()[10]);
  EXPECT_DOUBLE_EQ(-1.6, antex_satellite_data.GetPhaseCenterData(1).GetPhaseCenterVariationParameters_mm()[11]);
  EXPECT_DOUBLE_EQ(-1.1, antex_satellite_data.GetPhaseCenterData(1).GetPhaseCenterVariationParameters_mm()[12]);
  EXPECT_DOUBLE_EQ(0.0, antex_satellite_data.GetPhaseCenterData(1).GetPhaseCenterVariationParameters_mm()[13]);
  EXPECT_DOUBLE_EQ(1.5, antex_satellite_data.GetPhaseCenterData(1).GetPhaseCenterVariationParameters_mm()[14]);
  EXPECT_DOUBLE_EQ(2.2, antex_satellite_data.GetPhaseCenterData(1).GetPhaseCenterVariationParameters_mm()[15]);
  // TODO: Add all satellite check?
}
