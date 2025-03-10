// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


// gtest
#include <gtest/gtest.h>

// romea
#include "romea_core_localisation_imu/CheckupAttitude.hpp"

class TestAttitudeDiagnostic : public ::testing::Test
{
public:
  TestAttitudeDiagnostic()
  : frame(),
    diagnostic()
  {
  }

  romea::core::RollPitchCourseFrame frame;
  romea::core::CheckupAttitude diagnostic;
};

////-----------------------------------------------------------------------------
// TEST_F(TestAttitudeDiagnostic, checkStaleAfterInstantiation)
//{
//  EXPECT_EQ(diagnostic.getReport().diagnostics.empty(),TRUE);
//  EXPECT_EQ(diagnostic.getReport().info.empty(),TRUE);
//}

////-----------------------------------------------------------------------------
// TEST_F(TestAttitudeDiagnostic, checkStaleAfterReset)
//{
//  diagnostic.evaluate(frame);
//  diagnostic.reset();
//  EXPECT_EQ(diagnostic.getReport().diagnostics.empty(),TRUE);
//  EXPECT_EQ(diagnostic.getReport().info.empty(),TRUE);
//}

//-----------------------------------------------------------------------------
TEST_F(TestAttitudeDiagnostic, withGoodFrame)
{
  frame.rollAngle = 1.0472;
  frame.pitchAngle = 0.349;
  EXPECT_EQ(diagnostic.evaluate(frame), romea::core::DiagnosticStatus::OK);
  EXPECT_EQ(diagnostic.getReport().diagnostics.front().status, romea::core::DiagnosticStatus::OK);
  EXPECT_STREQ(diagnostic.getReport().diagnostics.front().message.c_str(), "Attitude is OK.");
  EXPECT_STREQ(diagnostic.getReport().info.at("roll").c_str(), "1.0472");
  EXPECT_STREQ(diagnostic.getReport().info.at("pitch").c_str(), "0.349");
}

//-----------------------------------------------------------------------------
TEST_F(TestAttitudeDiagnostic, withUnavailableRollAngle)
{
  frame.rollAngle = 1.7453;
  frame.pitchAngle = -0.156;
  EXPECT_EQ(diagnostic.evaluate(frame), romea::core::DiagnosticStatus::ERROR);
  EXPECT_EQ(diagnostic.getReport().diagnostics.front().status, romea::core::DiagnosticStatus::ERROR);
  EXPECT_STREQ(
    diagnostic.getReport().diagnostics.front().message.c_str(),
    "Attitude angles are out of range.");
  EXPECT_STREQ(diagnostic.getReport().info.at("roll").c_str(), "1.7453");
  EXPECT_STREQ(diagnostic.getReport().info.at("pitch").c_str(), "-0.156");
}

//-----------------------------------------------------------------------------
TEST_F(TestAttitudeDiagnostic, withUnavailablePitchAngle)
{
  frame.rollAngle = 0.05;
  frame.pitchAngle = -1.903;
  EXPECT_EQ(diagnostic.evaluate(frame), romea::core::DiagnosticStatus::ERROR);
  EXPECT_EQ(diagnostic.getReport().diagnostics.front().status, romea::core::DiagnosticStatus::ERROR);
  EXPECT_STREQ(
    diagnostic.getReport().diagnostics.front().message.c_str(),
    "Attitude angles are out of range.");
  EXPECT_STREQ(diagnostic.getReport().info.at("roll").c_str(), "0.05");
  EXPECT_STREQ(diagnostic.getReport().info.at("pitch").c_str(), "-1.903");
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
