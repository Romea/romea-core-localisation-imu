// gtest
#include <gtest/gtest.h>

// std
#include <random>

// romea
#include "romea_core_localisation_imu/AngularSpeedBias.hpp"

bool boolean(const romea::DiagnosticStatus & status)
{
  return status == romea::DiagnosticStatus::OK;
}


class TestAngularSpeedBias : public ::testing::Test
{
public:
  TestAngularSpeedBias():
    rate(50.),
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
    accelerations.accelerationAlongZAxis = accelerationDistribution(generator)+9.81;
  }

  void makeAngularSpeedFrame()
  {
    angularSpeeds.angularSpeedAroundXAxis = angularSpeedDistribution(generator);
    angularSpeeds.angularSpeedAroundYAxis = angularSpeedDistribution(generator);
    angularSpeeds.angularSpeedAroundZAxis = angularSpeedDistribution(generator);
  }

  void check(const romea::DiagnosticStatus & finalStatus,
             const std::string & finalMessage)
  {
    for (size_t n = 0 ;n < (5+2)*rate-2; ++n)
    {
      makeAccelerationFrame();
      makeAngularSpeedFrame();

      auto angularSpeedBias = angularSpeedBiasEstimator.evaluate(linearSpeed,
                                                                 accelerations,
                                                                 angularSpeeds);

      EXPECT_FALSE(angularSpeedBias.has_value());
      EXPECT_EQ(angularSpeedBiasEstimator.getReport().diagnostics.front().status,
                romea::DiagnosticStatus::WARN);
      EXPECT_STREQ(angularSpeedBiasEstimator.getReport().diagnostics.front().message.c_str(),
                  "Angular speed bias not available.");
    }

    makeAccelerationFrame();
    makeAngularSpeedFrame();

    auto angularSpeedBias = angularSpeedBiasEstimator.evaluate(linearSpeed,
                                                               accelerations,
                                                               angularSpeeds);

    EXPECT_EQ(angularSpeedBias.has_value(), boolean(finalStatus));
    EXPECT_EQ(angularSpeedBiasEstimator.getReport(). diagnostics.front().status, finalStatus);
    EXPECT_STREQ(angularSpeedBiasEstimator.getReport().diagnostics.front().message.c_str(),
                 finalMessage.c_str());
  }


  double rate;
  double accelerationStd;
  double angularSpeedStd;
  double angularSpeedBias;
  romea::AngularSpeedBias angularSpeedBiasEstimator;
  romea::AccelerationsFrame accelerations;
  romea::AngularSpeedsFrame angularSpeeds;
  double linearSpeed;

  std::default_random_engine generator;
  std::normal_distribution<double> accelerationDistribution;
  std::normal_distribution<double> angularSpeedDistribution;

  romea::DiagnosticReport report;
};

//-----------------------------------------------------------------------------
TEST_F(TestAngularSpeedBias, testAllOk)
{
  linearSpeed = 0;
  accelerationDistribution = std::normal_distribution<double>(0., accelerationStd);
  angularSpeedDistribution = std::normal_distribution<double>(0., angularSpeedStd);
  check(romea::DiagnosticStatus::OK, "Angular speed bias is OK.");
  report = angularSpeedBiasEstimator.getReport();
}

//-----------------------------------------------------------------------------
TEST_F(TestAngularSpeedBias, testWrongAccelerationStd)
{
  linearSpeed = 0;
  accelerationDistribution = std::normal_distribution<double>(0., 5*accelerationStd);
  angularSpeedDistribution = std::normal_distribution<double>(0., angularSpeedStd);
  check(romea::DiagnosticStatus::WARN, "Angular speed bias not available.");
}

//-----------------------------------------------------------------------------
TEST_F(TestAngularSpeedBias, testWrongAngularSpeedStd)
{
  linearSpeed = 0;
  accelerationDistribution = std::normal_distribution<double>(0., accelerationStd);
  angularSpeedDistribution = std::normal_distribution<double>(0., 5*angularSpeedStd);
  check(romea::DiagnosticStatus::WARN, "Angular speed bias not available.");
}

//-----------------------------------------------------------------------------
TEST_F(TestAngularSpeedBias, testLinearSpeedUpperToZero)
{
  linearSpeed = 1.0;
  accelerationDistribution = std::normal_distribution<double>(0., accelerationStd);
  angularSpeedDistribution = std::normal_distribution<double>(0., angularSpeedStd);
  check(romea::DiagnosticStatus::WARN, "Angular speed bias not available.");
}

//-----------------------------------------------------------------------------
TEST_F(TestAngularSpeedBias, testResetAfterAllOK)
{
  linearSpeed = 0;
  accelerationDistribution = std::normal_distribution<double>(0., accelerationStd);
  angularSpeedDistribution = std::normal_distribution<double>(0., angularSpeedStd);
  check(romea::DiagnosticStatus::OK, "Angular speed bias is OK.");

  angularSpeedBiasEstimator.reset(false);
  auto report = angularSpeedBiasEstimator.getReport();
  EXPECT_EQ(report.diagnostics.front().status, romea::DiagnosticStatus::STALE);
}

//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
