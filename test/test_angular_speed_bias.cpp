// gtest
#include <gtest/gtest.h>

//romea
#include "romea_localisation_imu/AngularSpeedBias.hpp"

//std
#include <random>

class TestAngularSpeedBias : public ::testing::Test
{
public:

  TestAngularSpeedBias():
    rate(50.),
    accelerationStd(0.001),
    angularSpeedStd(0.01),
    angularSpeedBias(0),
    angularSpeedBiasEstimator(rate,accelerationStd,angularSpeedStd),
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
    accelerations.accelerationAlongXAxis=accelerationDistribution(generator);
    accelerations.accelerationAlongYAxis=accelerationDistribution(generator);
    accelerations.accelerationAlongZAxis=accelerationDistribution(generator)+9.81;
  }

  void makeAngularSpeedFrame()
  {
    angularSpeeds.angularSpeedAroundXAxis=angularSpeedDistribution(generator);
    angularSpeeds.angularSpeedAroundYAxis=angularSpeedDistribution(generator);
    angularSpeeds.angularSpeedAroundZAxis=angularSpeedDistribution(generator)+9.81;
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
  accelerationDistribution= std::normal_distribution<double>(0.,accelerationStd);
  angularSpeedDistribution= std::normal_distribution<double>(0.,angularSpeedStd);

  for(size_t n=0;n<(5+2)*rate-2;++n)
  {
    makeAccelerationFrame();
    makeAngularSpeedFrame();
    EXPECT_FALSE(angularSpeedBiasEstimator.evaluate(linearSpeed,
                                                    accelerations,
                                                    angularSpeeds,
                                                    angularSpeedBias));

    EXPECT_EQ(angularSpeedBiasEstimator.getReport().diagnostics.front().status,romea::DiagnosticStatus::WARN);
    EXPECT_STREQ(angularSpeedBiasEstimator.getReport().diagnostics.front().message.c_str(),"Angular speed bias not available.");
  }

  makeAccelerationFrame();
  makeAngularSpeedFrame();
  EXPECT_TRUE(angularSpeedBiasEstimator.evaluate(linearSpeed,
                                                 accelerations,
                                                 angularSpeeds,
                                                 angularSpeedBias));

  EXPECT_EQ(angularSpeedBiasEstimator.getReport().diagnostics.front().status,romea::DiagnosticStatus::OK);
  EXPECT_STREQ(angularSpeedBiasEstimator.getReport().diagnostics.front().message.c_str(),"Angular speed bias is OK.");

}

//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
