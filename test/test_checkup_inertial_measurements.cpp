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

// std
#include <cmath>

// romea
#include "romea_core_localisation_imu/CheckupInertialMeasurements.hpp"

class TestInertialMeasurementsDiagnostic : public ::testing::Test
{
public:
  TestInertialMeasurementsDiagnostic()
  : accelerations({-1, 1, 9.81}),
    angularSpeeds({10 / 180. * M_PI, -20 / 180. * M_PI, 60 / 180. * M_PI}),
    diagnostic(10., 360 / 180 * M_PI)
  {
  }

  romea::AccelerationsFrame accelerations;
  romea::AngularSpeedsFrame angularSpeeds;
  romea::CheckupInertialMeasurements diagnostic;
};

//-----------------------------------------------------------------------------
TEST_F(TestInertialMeasurementsDiagnostic, checkGoodFrames)
{
  EXPECT_EQ(diagnostic.evaluate(accelerations, angularSpeeds), romea::DiagnosticStatus::OK);
  EXPECT_STREQ(
    diagnostic.getReport().diagnostics.front().message.c_str(),
    "Acceleration data is OK.");
  EXPECT_STREQ(
    diagnostic.getReport().diagnostics.back().message.c_str(),
    "Angular speed data is OK.");
  EXPECT_STREQ(diagnostic.getReport().info.at("acceleration_x").c_str(), "-1");
  EXPECT_STREQ(diagnostic.getReport().info.at("acceleration_y").c_str(), "1");
  EXPECT_STREQ(diagnostic.getReport().info.at("acceleration_z").c_str(), "9.81");
  EXPECT_STREQ(diagnostic.getReport().info.at("angular_speed_x").c_str(), "0.174533");
  EXPECT_STREQ(diagnostic.getReport().info.at("angular_speed_y").c_str(), "-0.349066");
  EXPECT_STREQ(diagnostic.getReport().info.at("angular_speed_z").c_str(), "1.0472");
}

//-----------------------------------------------------------------------------
TEST_F(TestInertialMeasurementsDiagnostic, checkAccelerationFrameOutOfRange)
{
  accelerations.accelerationAlongZAxis = 11;
  EXPECT_EQ(diagnostic.evaluate(accelerations, angularSpeeds), romea::DiagnosticStatus::ERROR);
  EXPECT_STREQ(
    diagnostic.getReport().diagnostics.front().message.c_str(),
    "Acceleration data is out of range.");
}

//-----------------------------------------------------------------------------
TEST_F(TestInertialMeasurementsDiagnostic, checkAngularSpeedFrameOutOfRange)
{
  angularSpeeds.angularSpeedAroundZAxis = 2 * 360 / 180. * M_PI;
  EXPECT_EQ(diagnostic.evaluate(accelerations, angularSpeeds), romea::DiagnosticStatus::ERROR);
  EXPECT_STREQ(
    diagnostic.getReport().diagnostics.back().message.c_str(),
    "Angular speed data is out of range.");
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
