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
#include <random>
#include <string>

// romea
#include "romea_core_localisation_imu/AngularSpeedBias.hpp"

bool boolean(const romea::core::DiagnosticStatus & status)
{
  return status == romea::core::DiagnosticStatus::OK;
}


class TestAngularSpeedBias : public ::testing::Test
{
public:
  TestAngularSpeedBias()
  : rate(50.),
    accelerationStd(0.001),
    angularSpeedStd(0.01),
    angularSpeedBias(0),
    angularSpeedBiasEstimator(rate, accelerationStd, angularSpeedStd),
    accelerations(),
    angularSpeeds(),
    linearSpeed(0.),
    generator(0),
    accelerationDistribution(),
    angularSpeedDistribution()
  {
  }

  void makeAccelerationFrame()
  {
    accelerations.accelerationAlongXAxis = accelerationDistribution(generator);
    accelerations.accelerationAlongYAxis = accelerationDistribution(generator);
    accelerations.accelerationAlongZAxis = accelerationDistribution(generator) + 9.81;
  }

  void makeAngularSpeedFrame()
  {
    angularSpeeds.angularSpeedAroundXAxis = angularSpeedDistribution(generator);
    angularSpeeds.angularSpeedAroundYAxis = angularSpeedDistribution(generator);
    angularSpeeds.angularSpeedAroundZAxis = angularSpeedDistribution(generator);
  }

  void check(
    const romea::core::DiagnosticStatus & finalStatus,
    const std::string & finalMessage)
  {
    for (size_t n = 0; n < (5 + 2) * rate - 2; ++n) {
      makeAccelerationFrame();
      makeAngularSpeedFrame();

      auto angularSpeedBias = angularSpeedBiasEstimator.evaluate(
        linearSpeed,
        accelerations,
        angularSpeeds);

      EXPECT_FALSE(angularSpeedBias.has_value());
      EXPECT_EQ(
        angularSpeedBiasEstimator.getReport().diagnostics.front().status,
        romea::core::DiagnosticStatus::WARN);
      EXPECT_STREQ(
        angularSpeedBiasEstimator.getReport().diagnostics.front().message.c_str(),
        "Angular speed bias not available.");
    }

    makeAccelerationFrame();
    makeAngularSpeedFrame();

    auto angularSpeedBias = angularSpeedBiasEstimator.evaluate(
      linearSpeed,
      accelerations,
      angularSpeeds);

    EXPECT_EQ(angularSpeedBias.has_value(), boolean(finalStatus));
    EXPECT_EQ(angularSpeedBiasEstimator.getReport().diagnostics.front().status, finalStatus);
    EXPECT_STREQ(
      angularSpeedBiasEstimator.getReport().diagnostics.front().message.c_str(),
      finalMessage.c_str());
  }


  double rate;
  double accelerationStd;
  double angularSpeedStd;
  double angularSpeedBias;
  romea::core::AngularSpeedBias angularSpeedBiasEstimator;
  romea::core::AccelerationsFrame accelerations;
  romea::core::AngularSpeedsFrame angularSpeeds;
  double linearSpeed;

  std::default_random_engine generator;
  std::normal_distribution<double> accelerationDistribution;
  std::normal_distribution<double> angularSpeedDistribution;

  romea::core::DiagnosticReport report;
};

//-----------------------------------------------------------------------------
TEST_F(TestAngularSpeedBias, testAllOk)
{
  linearSpeed = 0;
  accelerationDistribution = std::normal_distribution<double>(0., accelerationStd);
  angularSpeedDistribution = std::normal_distribution<double>(0., angularSpeedStd);
  check(romea::core::DiagnosticStatus::OK, "Angular speed bias is OK.");
  report = angularSpeedBiasEstimator.getReport();
}

//-----------------------------------------------------------------------------
TEST_F(TestAngularSpeedBias, testWrongAccelerationStd)
{
  linearSpeed = 0;
  accelerationDistribution = std::normal_distribution<double>(0., 5 * accelerationStd);
  angularSpeedDistribution = std::normal_distribution<double>(0., angularSpeedStd);
  check(romea::core::DiagnosticStatus::WARN, "Angular speed bias not available.");
}

//-----------------------------------------------------------------------------
TEST_F(TestAngularSpeedBias, testWrongAngularSpeedStd)
{
  linearSpeed = 0;
  accelerationDistribution = std::normal_distribution<double>(0., accelerationStd);
  angularSpeedDistribution = std::normal_distribution<double>(0., 5 * angularSpeedStd);
  check(romea::core::DiagnosticStatus::WARN, "Angular speed bias not available.");
}

//-----------------------------------------------------------------------------
TEST_F(TestAngularSpeedBias, testLinearSpeedUpperToZero)
{
  linearSpeed = 1.0;
  accelerationDistribution = std::normal_distribution<double>(0., accelerationStd);
  angularSpeedDistribution = std::normal_distribution<double>(0., angularSpeedStd);
  check(romea::core::DiagnosticStatus::WARN, "Angular speed bias not available.");
}

//-----------------------------------------------------------------------------
TEST_F(TestAngularSpeedBias, testResetAfterAllOK)
{
  linearSpeed = 0;
  accelerationDistribution = std::normal_distribution<double>(0., accelerationStd);
  angularSpeedDistribution = std::normal_distribution<double>(0., angularSpeedStd);
  check(romea::core::DiagnosticStatus::OK, "Angular speed bias is OK.");

  angularSpeedBiasEstimator.reset(false);
  auto report = angularSpeedBiasEstimator.getReport();
  EXPECT_EQ(report.diagnostics.front().status, romea::core::DiagnosticStatus::STALE);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
