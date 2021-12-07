// gtest
#include <gtest/gtest.h>

//romea
#include "romea_core_localisation_imu/CheckupAttitude.hpp"

class TestAttitudeDiagnostic : public ::testing::Test
{
public:

  TestAttitudeDiagnostic():
    frame(),
    diagnostic()
  {
  }

  romea::RollPitchCourseFrame frame;
  romea::CheckupAttitude diagnostic;
};

//-----------------------------------------------------------------------------
TEST_F(TestAttitudeDiagnostic, withGoodFrame)
{
  frame.rollAngle=1.0472;
  frame.pitchAngle= 0.349;
  EXPECT_EQ(diagnostic.evaluate(frame),romea::DiagnosticStatus::OK);
  EXPECT_EQ(diagnostic.getReport().diagnostics.front().status,romea::DiagnosticStatus::OK);
  EXPECT_STREQ(diagnostic.getReport().diagnostics.front().message.c_str(),"Attitude is OK.");
  EXPECT_STREQ(diagnostic.getReport().info.at("roll").c_str(),"1.0472");
  EXPECT_STREQ(diagnostic.getReport().info.at("pitch").c_str(),"0.349");
}

//-----------------------------------------------------------------------------
TEST_F(TestAttitudeDiagnostic, withUnavailableRollAngle)
{
  frame.rollAngle=1.7453;
  frame.pitchAngle=-0.156;
  EXPECT_EQ(diagnostic.evaluate(frame),romea::DiagnosticStatus::ERROR);
  EXPECT_EQ(diagnostic.getReport().diagnostics.front().status,romea::DiagnosticStatus::ERROR);
  EXPECT_STREQ(diagnostic.getReport().diagnostics.front().message.c_str(),"Attitude angles are out of range.");
  EXPECT_STREQ(diagnostic.getReport().info.at("roll").c_str(),"1.7453");
  EXPECT_STREQ(diagnostic.getReport().info.at("pitch").c_str(),"-0.156");
}

//-----------------------------------------------------------------------------
TEST_F(TestAttitudeDiagnostic, withUnavailablePitchAngle)
{
  frame.rollAngle=0.05;
  frame.pitchAngle=-1.903;
  EXPECT_EQ(diagnostic.evaluate(frame),romea::DiagnosticStatus::ERROR);
  EXPECT_EQ(diagnostic.getReport().diagnostics.front().status,romea::DiagnosticStatus::ERROR);
  EXPECT_STREQ(diagnostic.getReport().diagnostics.front().message.c_str(),"Attitude angles are out of range.");
  EXPECT_STREQ(diagnostic.getReport().info.at("roll").c_str(),"0.05");
  EXPECT_STREQ(diagnostic.getReport().info.at("pitch").c_str(),"-1.903");
}


//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
