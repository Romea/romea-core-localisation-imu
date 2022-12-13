// gtest
#include <gtest/gtest.h>

// std
#include <random>

// romea
#include <romea_core_localisation_imu/LocalisationIMUPlugin.hpp>

bool boolean(const romea::DiagnosticStatus & status)
{
  return status == romea::DiagnosticStatus::OK;
}

class TestIMUPlugin : public ::testing::Test
{
public:
  TestIMUPlugin():
    generator(0),
    attitudeX(0),
    attitudeY(0),
    attitudeZ(0),
    attitudeDistribution(0, 0.01745),
    accelerationX(0),
    accelerationY(0),
    accelerationZ(0),
    accelerationDistribution(0, 0.0005*std::sqrt(10)),
    angularSpeedX(0),
    angularSpeedY(0),
    angularSpeedZ(0),
    angularSpeedDistribution(0, 3.4907e-04*std::sqrt(10)/180.*M_PI),
    linearSpeed(0.0),
    angularSpeedObs(),
    attitudeObs(),
    plugin(nullptr)
  {
  }

  void SetUp() override
  {
    auto imu = std::make_unique<romea::IMUAHRS>(
      10,
      0.0005, 0.02, 10.,
      3.4907e-04/180.*M_PI, 3.4907e-02/180.*M_PI, 300./180.*M_PI,
      7.e-09, 1.e-08, 0.000075,
      0.01745);

    plugin = std::make_unique<romea::LocalisationIMUPlugin>(std::move(imu));
  }

  const romea::Diagnostic & diagnostic(const size_t & index)
  {
    return *std::next(std::cbegin(report.diagnostics), index);
  }

  void step(const size_t & n,
            const romea::DiagnosticStatus & attitudeStatus,
            const romea::DiagnosticStatus & angularBiasStatus)
  {
    //    std::cout << " step n "<< n<< std::endl;
    if (std::isfinite(linearSpeed))
    {
      romea::Duration sstamp = romea::durationFromSecond(n/10.);
      plugin->processLinearSpeed(sstamp, linearSpeed);
    }

    romea::Duration stamp = romea::durationFromSecond(0.1+n/10.);

    EXPECT_EQ(plugin->computeAngularSpeed(stamp,
                                          accelerationX+accelerationDistribution(generator),
                                          accelerationY+accelerationDistribution(generator),
                                          accelerationZ+accelerationDistribution(generator),
                                          angularSpeedX+angularSpeedDistribution(generator),
                                          angularSpeedY+angularSpeedDistribution(generator),
                                          angularSpeedZ+angularSpeedDistribution(generator),
                                          angularSpeedObs), boolean(angularBiasStatus));

    EXPECT_EQ(plugin->computeAttitude(stamp,
                                      attitudeX+attitudeDistribution(generator),
                                      attitudeY+attitudeDistribution(generator),
                                      attitudeZ+attitudeDistribution(generator),
                                      attitudeObs), boolean(attitudeStatus));


    report = plugin->makeDiagnosticReport(stamp);

    //    for(auto diag : report.diagnostics)
    //    {
    //      std::cout << diag.status <<" "<< diag.message << std::endl;
    //    }
  }

  void check(const romea::DiagnosticStatus & finalLinearSpeedStatus,
             const romea::DiagnosticStatus & finalAccelerationStatus,
             const romea::DiagnosticStatus & finalAngularSpeedStatus,
             const romea::DiagnosticStatus & finalAttitudeStatus,
             const romea::DiagnosticStatus & finalAngularBiasStatus)
  {
    for (size_t n=0; n < 20; ++n)
    {
      step(n, romea::DiagnosticStatus::ERROR, romea::DiagnosticStatus::ERROR);
      EXPECT_EQ(report.diagnostics.size(), 3);
      EXPECT_EQ(diagnostic(0).status, romea::DiagnosticStatus::ERROR);
      EXPECT_EQ(diagnostic(1).status, romea::DiagnosticStatus::ERROR);
      EXPECT_EQ(diagnostic(2).status, romea::DiagnosticStatus::ERROR);
    }

    for (size_t n=20; n < 88; ++n)
    {
      step(n, finalAttitudeStatus, romea::DiagnosticStatus::ERROR);

      EXPECT_EQ(report.diagnostics.size(),
                6+int(finalAngularBiasStatus != romea::DiagnosticStatus::STALE));
      EXPECT_EQ(diagnostic(0).status, std::isfinite(linearSpeed) ?
                romea::DiagnosticStatus::OK : romea::DiagnosticStatus::ERROR);
      EXPECT_EQ(diagnostic(1).status, romea::DiagnosticStatus::OK);  // atttiude rate
      EXPECT_EQ(diagnostic(2).status, finalAttitudeStatus);  // atttiude
      EXPECT_EQ(diagnostic(3).status, romea::DiagnosticStatus::OK);  // inertial rate
      EXPECT_EQ(diagnostic(4).status, finalAccelerationStatus);  // acceleration
      EXPECT_EQ(diagnostic(5).status, finalAngularSpeedStatus);  // angular speeds

      if (finalAngularBiasStatus != romea::DiagnosticStatus::STALE)
      {
        EXPECT_EQ(diagnostic(6).status, romea::DiagnosticStatus::WARN);  // angular speed_bias
      }
    }

    step(88, finalAttitudeStatus, finalAngularBiasStatus);
    EXPECT_EQ(report.diagnostics.size(),
              6+int(finalAngularBiasStatus!=romea::DiagnosticStatus::STALE));
    EXPECT_EQ(diagnostic(0).status, std::isfinite(linearSpeed) ?
              romea::DiagnosticStatus::OK : romea::DiagnosticStatus::ERROR);
    EXPECT_EQ(diagnostic(1).status, romea::DiagnosticStatus::OK);  // atttiude rate
    EXPECT_EQ(diagnostic(2).status, finalAttitudeStatus);  // atttiude
    EXPECT_EQ(diagnostic(3).status, romea::DiagnosticStatus::OK);  // inertial rate
    EXPECT_EQ(diagnostic(4).status, finalAccelerationStatus);  // acceleration
    EXPECT_EQ(diagnostic(5).status, finalAngularSpeedStatus);  // angular speeds

    if (finalAngularBiasStatus != romea::DiagnosticStatus::STALE)
    {
      EXPECT_EQ(diagnostic(6).status, finalAngularBiasStatus);  // angular speed_bias
    }
  }

  std::default_random_engine generator;
  double attitudeX, attitudeY, attitudeZ;
  std::normal_distribution<double> attitudeDistribution;
  double accelerationX, accelerationY, accelerationZ;
  std::normal_distribution<double> accelerationDistribution;
  double angularSpeedX, angularSpeedY, angularSpeedZ;
  std::normal_distribution<double> angularSpeedDistribution;
  double  linearSpeed;

  romea::ObservationAngularSpeed angularSpeedObs;
  romea::ObservationAttitude attitudeObs;
  std::unique_ptr<romea::LocalisationIMUPlugin> plugin;

  romea::DiagnosticReport report;
};




//-----------------------------------------------------------------------------
TEST_F(TestIMUPlugin, testAllOk)
{
  check(romea::DiagnosticStatus::OK, //finalLinearSpeedStatus
        romea::DiagnosticStatus::OK,//finalAccelerationStatus
        romea::DiagnosticStatus::OK,//finalAngularSpeedStatus
        romea::DiagnosticStatus::OK,//finalAttitudeStatus
        romea::DiagnosticStatus::OK);//finalAngularBiasStatus
}

//-----------------------------------------------------------------------------
TEST_F(TestIMUPlugin, testAttitudeOutOfRange)
{
  attitudeX = M_PI;
  check(romea::DiagnosticStatus::OK, //finalLinearSpeedStatus
        romea::DiagnosticStatus::OK,//finalAccelerationStatus
        romea::DiagnosticStatus::OK,//finalAngularSpeedStatus
        romea::DiagnosticStatus::ERROR,//finalAttitudeStatus
        romea::DiagnosticStatus::OK);//finalAngularBiasStatus
}

//-----------------------------------------------------------------------------
TEST_F(TestIMUPlugin, testAngularSpeedOutOfRange)
{
  angularSpeedZ= 2*M_PI;
  check(romea::DiagnosticStatus::OK, //finalLinearSpeedStatus
        romea::DiagnosticStatus::OK,//finalAccelerationStatus
        romea::DiagnosticStatus::ERROR,//finalAngularSpeedStatus
        romea::DiagnosticStatus::OK,//finalAttitudeStatus
        romea::DiagnosticStatus::STALE);//finalAngularBiasStatus
}

//-----------------------------------------------------------------------------
TEST_F(TestIMUPlugin, testAccelerationOutOfRange)
{
  accelerationZ= 20;
  check(romea::DiagnosticStatus::OK, //finalLinearSpeedStatus
        romea::DiagnosticStatus::ERROR,//finalAccelerationStatus
        romea::DiagnosticStatus::OK,//finalAngularSpeedStatus
        romea::DiagnosticStatus::OK,//finalAttitudeStatus
        romea::DiagnosticStatus::STALE);//finalAngularBiasStatus
}

//-----------------------------------------------------------------------------
TEST_F(TestIMUPlugin, testLinearSpeedNotEqualZero)
{
  linearSpeed= 1;
  check(romea::DiagnosticStatus::OK, //finalLinearSpeedStatus
        romea::DiagnosticStatus::OK,//finalAccelerationStatus
        romea::DiagnosticStatus::OK,//finalAngularSpeedStatus
        romea::DiagnosticStatus::OK,//finalAttitudeStatus
        romea::DiagnosticStatus::WARN);//finalAngularBiasStatus
}

//-----------------------------------------------------------------------------
TEST_F(TestIMUPlugin, testAngularSpeedStdToHigh)
{
  angularSpeedDistribution = std::normal_distribution<double>(0,100*3.4907e-04*std::sqrt(10)/180.*M_PI);
  check(romea::DiagnosticStatus::OK, //finalLinearSpeedStatus
        romea::DiagnosticStatus::OK,//finalAccelerationStatus
        romea::DiagnosticStatus::OK,//finalAngularSpeedStatus
        romea::DiagnosticStatus::OK,//finalAttitudeStatus
        romea::DiagnosticStatus::WARN);//finalAngularBiasStatus
}

//-----------------------------------------------------------------------------
TEST_F(TestIMUPlugin, testAccelerationStdToHigh)
{
  accelerationDistribution = std::normal_distribution<double>(0,100*0.0005*std::sqrt(10));
  check(romea::DiagnosticStatus::OK, //finalLinearSpeedStatus
        romea::DiagnosticStatus::OK,//finalAccelerationStatus
        romea::DiagnosticStatus::OK,//finalAngularSpeedStatus
        romea::DiagnosticStatus::OK,//finalAttitudeStatus
        romea::DiagnosticStatus::WARN);//finalAngularBiasStatus
}

//-----------------------------------------------------------------------------
TEST_F(TestIMUPlugin, testTimeout)
{
  check(romea::DiagnosticStatus::OK, //finalLinearSpeedStatus
        romea::DiagnosticStatus::OK,//finalAccelerationStatus
        romea::DiagnosticStatus::OK,//finalAngularSpeedStatus
        romea::DiagnosticStatus::OK,//finalAttitudeStatus
        romea::DiagnosticStatus::OK);//finalAngularBiasStatus

  report = plugin->makeDiagnosticReport(romea::durationFromSecond(10.));

//  for(auto diag : report.diagnostics)
//  {
//    std::cout << diag.status <<" "<< diag.message << std::endl;
//  }

//  for(auto info : report.info)
//  {
//    std::cout << info.first <<" "<< info.second<< std::endl;
//  }
}

//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
