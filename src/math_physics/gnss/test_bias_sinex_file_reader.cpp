#include <gtest/gtest.h>

#include "bias_sinex_file_reader.hpp"

TEST(BiasSinex, Constructor) {
  // File read error check
  BiasSinexFileReader bias_sinex_file_fault("false_file_path.BSX");
  EXPECT_FALSE(bias_sinex_file_fault.GetFileReadSuccessFlag());

  // File read check
  std::string test_file_name = "/src/library/gnss/example.BSX";
  BiasSinexFileReader bias_sinex_file(CORE_DIR_FROM_EXE + test_file_name);
  EXPECT_TRUE(bias_sinex_file.GetFileReadSuccessFlag());

  // Check data
  EXPECT_EQ(3234, bias_sinex_file.GetNumberOfBiasData());

  // Check first data
  BiasSolutionData bias_solution = bias_sinex_file.GetBiasData(0);
  EXPECT_EQ(BiasIdentifier::kDsb, bias_solution.GetIdentifier());
  EXPECT_EQ(BiasTargetSignal::kP1P2, bias_solution.GetTargetSignal());
  EXPECT_EQ(BiasUnit::kNs, bias_solution.GetUnit());
  EXPECT_NEAR(-6.75076755510313E+00, bias_solution.GetBias(), 1e-10);
  EXPECT_NEAR(2.642146E-01, bias_solution.GetBiasStandardDeviation(), 1e-5);

  // Check final data
  bias_solution = bias_sinex_file.GetBiasData(3234 - 1);
  EXPECT_EQ(BiasIdentifier::kIsb, bias_solution.GetIdentifier());
  EXPECT_EQ(BiasTargetSignal::kP1P2, bias_solution.GetTargetSignal());
  EXPECT_EQ(BiasUnit::kNs, bias_solution.GetUnit());
  EXPECT_NEAR(0.0, bias_solution.GetBias(), 1e-10);
  EXPECT_NEAR(0.0, bias_solution.GetBiasStandardDeviation(), 1e-6);
}
